#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseStamped
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
    global qr_next
    qr_next = msg.pose.position


# update callback, msg is string we need to parse with information on current qr in unknown frame, next qr in unknown frame, and code message
def msg_callback(msg):
    global qr_msg
    qr_msg = msg


def wandering(g_range_ahead, cmd_vel_pub):
    rospy.loginfo(g_range_ahead)
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
qr_next = None  # coords for next qr target in world
qr_msg = [None] * 5

# init subscribers and publishers
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)
qr_pos_sub = rospy.Subscriber(
    "visp_auto_tracker/object_position", PoseStamped, pose_callback
)
qr_msg_sub = rospy.Subscriber("visp_auto_tracker/code_message", String, msg_callback)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1, latch=False)

while not rospy.is_shutdown():
    if not qr_next:
        wandering(g_range_ahead, cmd_vel_pub)
        # check for next qr and solve for unknown world frame, the npublish transformation
    else:
        move_to_next(qr_next, cmd_vel_pub)

    rate.sleep()

if rospy.is_shutdown():
    print("a")
    twist = Twist()
    print("b")
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    cmd_vel_pub.publish(twist)
    print("c")
    rospy.spin()
