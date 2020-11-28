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


KEYPOINTS = [
    [[-4.43386650085, 2.58987593651, 0], [0, 0, -0.454587318297, 0.890702178084]],
    [[-3.94284963608, -2.56628060341, 0], [0, 0, 0.716695789534, 0.6973859371]],
    [[3.80292606354, 0.575657725334, 0], [0, 0, 0, 1]],
    [[6.74980449677, 2.69655823708, 0], [0, 0, -0.773033822559, 0.634364807646]],
]
DIST_THRESH = 2
MAP_BOUNDS = [[-4, 4], [-7.5, 7.5]]


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
        global stop_driving, qr_dict, qr_curr_cam
        if qr_curr_cam is None or qr_curr_cam[2] >= DIST_THRESH:
            stop_driving = False
            # rospy.loginfo("Found QR but too far for good measurement")
            return

        str_split = msg.data.split("\r\n")
        split = [s.split("=")[1] for s in str_split]
        idx = float(split[-2])

        if idx in qr_dict:
            stop_driving = False
            # rospy.loginfo("Found QR {} again".format(idx))
            return
        else:
            stop_driving = True
            rospy.loginfo("Found QR {}".format(int(idx)))
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

            R = tf.transformations.quaternion_matrix(quat)
            T = tf.transformations.translation_matrix(trans)
            H = tf.transformations.concatenate_matrices(T, R)
            qr_curr_map = np.dot(H, qr_curr_cam)
            qr_dict[idx] = {}
            qr_dict[idx]["curr_hidden"] = [float(split[0]), float(split[1])]
            qr_dict[idx]["next_pos"] = [float(split[2]), float(split[3])]
            qr_dict[idx]["char"] = split[-1]
            qr_dict[idx]["curr_map"] = qr_curr_map

            rospy.loginfo(
                "Added QR {} to dictionary with measured position:{}".format(
                    int(idx), qr_curr_map
                )
            )
            rospy.loginfo("Dictionary size = {}".format(len(qr_dict.keys())))

            stop_driving = False
            global client
            client.cancel_all_goals()


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


def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = "map"
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose


def goal_position(position):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = "map"
    goal_pose.target_pose.pose.position.x = position[0]
    goal_pose.target_pose.pose.position.y = position[1]
    goal_pose.target_pose.pose.position.z = 0.0
    goal_pose.target_pose.pose.orientation.x = 0
    goal_pose.target_pose.pose.orientation.y = 0
    goal_pose.target_pose.pose.orientation.z = 0
    goal_pose.target_pose.pose.orientation.w = 1.0

    return goal_pose


# init rospy things
rospy.init_node("track_qr_codes")
rate = rospy.Rate(60)
rospy.on_shutdown(shutdown_callback)

g_range_ahead = 1  # anything to start
qr_dict = {}
qr_curr_cam = None
qr_hunting = False
stop_driving = False

# # init actionlib client
client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
client.wait_for_server()

# init tf transforms
listener = tf.TransformListener()
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

# move to a good position for QR codes
# pose = [[-3.18581056595, 0.361307609081, 0.0], [0, 0, -0.718677107716, 0.695343954345]]
# goal = goal_pose(pose)
# client.send_goal(goal)
# client.wait_for_result()
key_idx = 0
while not rospy.is_shutdown():
    if not qr_hunting:
        if not stop_driving:
            # wandering(g_range_ahead, cmd_vel_pub)
            goal = goal_pose(KEYPOINTS[key_idx])
            key_idx = key_idx + 1 if key_idx < len(KEYPOINTS) else 0
            client.send_goal(goal)
            client.wait_for_result()
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
            hidden_in_world = (
                np.add(
                    np.subtract(p1w, np.dot(rotz(theta), p1h)),
                    np.subtract(p2w, np.dot(rotz(theta), p2h)),
                )
                / 2
            )
            rospy.loginfo(
                "Transformation from [w] to [h]\t angle:{:.2f}\t position:{}".format(
                    theta * 180 / np.pi, hidden_in_world
                )
            )

            t = Twist()
            cmd_vel_pub.publish(t)

            rospy.loginfo("Starting phase 2")
    else:
        keys = list(qr_dict.keys())
        for key in keys:
            next_idx = key + 1 if key < 5 else 1
            if next_idx not in qr_dict:
                # move to next qr given by next
                next_pos_hidden = np.array(qr_dict[key]["next_pos"])
                next_pos_world = np.add(
                    np.dot(rotz(theta), next_pos_hidden), hidden_in_world
                )
                # rospy.loginfo(
                #     "theta:{}\t next pos hidden:{}\t next_post_world:{}\t hidden in world:{}".format(
                #         theta * 180 / np.pi,
                #         next_pos_hidden,
                #         next_pos_world,
                #         hidden_in_world,
                #     )
                # )
                rospy.loginfo(
                    "Next is QR {} at {}".format(int(next_idx), next_pos_world)
                )

                # only go 75% of the way
                # rob_pos = rospy.wait_for_message(
                #     "amcl_pose", PoseWithCovarianceStamped()
                # ).pose.pose.position
                # rob_pos = np.array([rob_pos.x, rob_pos.y])
                # delta_pos = np.subtract(rob_pos, next_pos_world)
                # next_pos_world = np.add(rob_pos, 0.75 * delta_pos)
                # rospy.loginfo("Going to {}".format(next_pos_world))

                # send action client to new coordiante
                goal = goal_position(next_pos_world)  # goal_pose(qr_transformed pose)
                client.send_goal(goal)
                client.wait_for_result()
                rospy.loginfo("Couldn't find QR en route, checking area")
                t = Twist()
                t.angular.z = -0.4
                cmd_vel_pub.publish(t)
                rospy.sleep(10)
        if len(qr_dict) == 5:
            rospy.loginfo("Finsihed")
            msg = []
            for i in range(5):
                msg.append(qr_dict[i + 1]["char"])
            rospy.loginfo("Message is {}".format(msg))
            break