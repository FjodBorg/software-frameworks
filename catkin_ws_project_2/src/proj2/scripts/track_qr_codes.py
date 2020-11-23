#!/usr/bin/env python
import numpy as np
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan


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
    str_split = msg.data.split("\r\n")
    rospy.loginfo("Found QR {}".format(str_split))
    split = [s.split("=")[1] for s in str_split]
    global qr_dict
    idx = float(split[-2])
    qr_dict[(idx, "curr_hidden")] = [float(split[0]), float(split[1])]
    qr_dict[(idx, "next")] = [float(split[2]), float(split[3])]
    qr_dict[(idx, "char")] = split[-1]

    global listener
    try:
        listener.waitForTransform(
            "/map", "/camera_optical_link", rospy.Time(0), rospy.Time(1)
        )

        (trans, quat) = listener.lookupTransform(
            "/map", "/camera_optical_link", rospy.Time(0)
        )
    except:
        rospy.loginfo("No transform")
        return

    R = tf.transformations.quaternion_matrix(quat)
    T = tf.transformations.translation_matrix(trans)
    H = tf.transformations.concatenate_matrices(T, R)
    qr_curr_map = np.dot(H, qr_curr_cam)
    qr_dict[(idx, "curr_map")] = qr_curr_map

    qr_curr_hidden = np.array(qr_dict[(idx, "curr")])
    rospy.loginfo("map: {}\t hidden:{}".format(qr_curr_map[0:2], qr_curr_hidden))


def wandering(g_range_ahead, cmd_vel_pub):
    # rospy.loginfo(g_range_ahead)
    if g_range_ahead < 0.8:
        # TURN
        driving_forward = False
        print("Turn")

    else:  # we're not driving_forward
        driving_forward = True  # we're done spinning, time to go forward!
        # DRIVE
        print("Drive")

    twist = Twist()
    if driving_forward:
        twist.linear.x = 0.4
        twist.angular.z = 0.0
    else:
        twist.linear.x = 0.0
        twist.angular.z = 0.4
    cmd_vel_pub.publish(twist)


def move_to_next(qr_next, cmd_vel_pub):
    return 0


def check_for_qr():
    return 0


# init rospy things
rospy.init_node("track_qr_codes")
rate = rospy.Rate(60)
rospy.on_shutdown(shutdown_callback)

g_range_ahead = 1  # anything to start
qr_idx = 0
qr_dict = {}
qr_curr_cam = None
qr_found = False  # coords for next qr target in world

# init subscribers and publishers
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)
qr_pos_sub = rospy.Subscriber(
    "visp_auto_tracker/object_position", PoseStamped, pose_callback
)
qr_msg_sub = rospy.Subscriber("visp_auto_tracker/code_message", String, msg_callback)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1, latch=False)

# init tf transforms
listener = tf.TransformListener()
hidden_to_world = []
rospy.sleep(1)

while not rospy.is_shutdown():
    if not qr_found:
        # wandering(g_range_ahead, cmd_vel_pub)
        # # check for next qr and solve for unknown world frame, the npublish transformation
        if qr_curr_cam is not None and len(qr_dict) > 1:
            # qr_found = True
            (trans, quat) = listener.lookupTransform(
                "/map", "/camera_optical_link", rospy.Time(0)
            )
            R = tf.transformations.quaternion_matrix(quat)
            T = tf.transformations.translation_matrix(trans)
            H = tf.transformations.concatenate_matrices(T, R)
            # rospy.loginfo("H:{}\t qr_curr_cam:{}".format(H, qr_curr_cam))
            qr_curr_map = np.dot(H, qr_curr_cam)

            idx = qr_dict.keys()[0][0]
            qr_curr_hidden = np.array(qr_dict[(idx, "curr")])
            rospy.loginfo(
                "map: {}\t hidden:{}".format(qr_curr_map[0:2], qr_curr_hidden)
            )
            hidden_in_map = np.subtract(qr_curr_map[0:2], qr_curr_hidden)
            rospy.loginfo("Hidden frame origin in map frame: {}".format(hidden_in_map))
    else:
        pass
        # move_to_next(qr_next, cmd_vel_pub)

    rate.sleep()
