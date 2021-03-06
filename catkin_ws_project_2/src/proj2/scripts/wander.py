#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
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


g_range_ahead = 1  # anything to start
scan_sub = rospy.Subscriber("scan", LaserScan, scan_callback)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1, latch=False)
rospy.init_node("wander")
state_change_time = rospy.Time.now() + rospy.Duration(1)
driving_forward = True
rate = rospy.Rate(60)

rospy.on_shutdown(shutdown_callback)

while not rospy.is_shutdown():
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
