#!/usr/bin/env python
import numpy as np
import rospy
import tf
import actionlib
import random

from std_msgs.msg import String
from geometry_msgs.msg import (
    Twist,
    Pose,
    PoseStamped,
    TransformStamped,
    PoseWithCovarianceStamped,
)
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def shutdown_callback():
    cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1, latch=False)
    twist = Twist()
    cmd_vel_pub.publish(twist)


def scan_callback(msg):
    global g_range_ahead
    tmp = [msg.ranges[0]]
    for i in range(1, 21):
        tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges) - 21, len(msg.ranges)):
        tmp.append(msg.ranges[i])
    g_range_ahead = min(tmp)


# update callback, msg is camera-frame QR code position
def pose_callback(msg):
    global qr_curr_cam
    qr_curr_cam = np.array(
        [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 1.0]
    )
    # qr_curr_cam = msg.pose


# update callback, msg is string we need to parse with information on current qr in unknown frame, next qr in unknown frame, and code message
def msg_callback(msg):
    if msg.data != "":
        global stop_driving, qr_dict

        str_split = msg.data.split("\r\n")
        split = [s.split("=")[1] for s in str_split]
        idx = float(split[-2])

        if idx in qr_dict:
            stop_driving = False
            # rospy.loginfo("Found QR {} again".format(idx))
            return
        else:
            stop_driving = True
            rospy.loginfo("Found new QR {}".format(idx))
            global listener
            try:
                now = rospy.Time.now()
                listener.waitForTransform(
                    "/map", "/camera_optical_link", now, rospy.Duration(10)
                )

                (trans, quat) = listener.lookupTransform(
                    "/map", "/camera_optical_link", now
                )
            except:
                rospy.loginfo("No transform")
                stop_driving = False
                return
            qr_dict[idx] = {}
            qr_dict[idx]["curr_hidden"] = [float(split[0]), float(split[1])]
            qr_dict[idx]["next"] = [float(split[2]), float(split[3])]
            qr_dict[idx]["char"] = split[-1]

            R = tf.transformations.quaternion_matrix(quat)
            T = tf.transformations.translation_matrix(trans)
            H = tf.transformations.concatenate_matrices(T, R)
            qr_curr_map = np.dot(H, qr_curr_cam)
            qr_dict[idx]["curr_map"] = qr_curr_map

            rospy.loginfo("Added QR {} to dictionary.".format(idx))
            rospy.loginfo("Dictionary size = {}".format(len(qr_dict.keys())))

            stop_driving = False


def wandering(g_range_ahead, cmd_vel_pub):
    # rospy.loginfo(g_range_ahead)
    if g_range_ahead < 0.8:
        # TURN
        driving_forward = False
        # print("Turn")

    else:  # we're not driving_forward
        driving_forward = True  # we're done spinning, time to go forward!
        # DRIVE
        # print("Drive")

    twist = Twist()
    if driving_forward:
        twist.linear.x = 0.4
        twist.angular.z = 0.0
    else:
        twist.linear.x = 0.0
        twist.angular.z = -0.4
    cmd_vel_pub.publish(twist)


def rotz(angle):
    cos = np.cos
    sin = np.sin
    return np.array([[cos(angle), -sin(angle)], [sin(angle), cos(angle)]])

# Used for waypoint navigation
def goal_pose(pose):  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'curr_map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
 
    return goal_pose


# init rospy things
rospy.init_node("track_qr_codes")
rate = rospy.Rate(60)
rospy.on_shutdown(shutdown_callback)

g_range_ahead = 1  # anything to start
qr_idx = 0
qr_dict = {}
qr_curr_cam = None
qr_hunting = False
stop_driving = False

rob_curr_world = Pose()

# # init actionlib client
client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
# client.wait_for_server()

# init tf transforms
listener = tf.TransformListener()
hidden_to_world = []
rospy.sleep(1)

# init subscribers and publishers
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)
qr_pos_sub = rospy.Subscriber(
    "visp_auto_tracker/object_position", PoseStamped, pose_callback
)
qr_msg_sub = rospy.Subscriber("visp_auto_tracker/code_message", String, msg_callback)
# amcl_pos_sub = rospy.Subscriber(
#     "amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback
# )
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1, latch=False)

while not rospy.is_shutdown():
    if not qr_hunting:
        if not stop_driving:
            wandering(g_range_ahead, cmd_vel_pub)
        else:
            t = Twist()
            cmd_vel_pub.publish(t)
        if len(qr_dict.keys()) >= 2:
            qr_hunting = True
            keys = list(qr_dict.keys())
            p1w = np.array(qr_dict[keys[0]]["curr_map"][0:2])
            p2w = np.array(qr_dict[keys[1]]["curr_map"][0:2])
            p1h = np.array(qr_dict[keys[0]]["curr_hidden"][0:2])
            p2h = np.array(qr_dict[keys[1]]["curr_hidden"][0:2])

            a = np.subtract(p1w, p2w)
            b = np.subtract(p1h, p2h)
            theta = np.arccos(
                np.dot(a, b) / np.linalg.norm(a, ord=2) / np.linalg.norm(b, ord=2)
            )
            # rospy.loginfo("a{}\t b{}\t theta{}".format(a, b, theta))
            hidden_in_world = np.subtract(p1w, np.dot(rotz(theta), p1h))
            rospy.loginfo(
                "Transformation from [w] to [h]\t angle:{0:.2f}\t position:{1:.2f}".format(
                    theta * 180 / np.pi, hidden_in_world
                )
            )

            t = Twist()
            cmd_vel_pub.publish(t)
    else:
        rospy.loginfo("Starting phase 2")
        keys = list(qr_dict.keys())
        for key in keys:
            next_idx = qr_dict[key]["next"]
            if next_idx not in qr_dict:
                # move to next qr given by next
                rospy.loginfo("Going to QR {}".format())
                """ to do"""
                """ calculat transformation"""
			
		pose = [[( , , ), ( , , , )],[( , , ), ( , , , )]]
                """send action client to new coordiante"""
        	goal = goal_pose(pose) # goal_pose(qr_transformed pose)
        	client.send_goal(goal)
        	client.wait_for_result()
	        rospy.sleep(3)
