#!/usr/bin/env python
from math import ceil
from numpy import zeros, array, linspace
from sensor_msgs.msg import JointState
import shape_msgs.msg as shape_msgs
import geometry_msgs.msg
import moveit_msgs.msg
import moveit_commander
import tf_conversions
import rospy
import copy
import sys
import roslib
roslib.load_manifest('proj1')


currentJointState = JointState()


def jointStatesCallback(msg):
    global currentJointState
    currentJointState = msg


if __name__ == '__main__':
    rospy.init_node('test_publish')

    # Setup subscriber
    #rospy.Subscriber("/joint_states", JointState, jointStatesCallback)

    pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)

    currentJointState = rospy.wait_for_message("/joint_states", JointState)
    rospy.loginfo('Received!')
    currentJointState.header.stamp = rospy.get_rostime()
    tmp = 0.7
    #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
    currentJointState.position = tuple(
        list(currentJointState.position[:6]) + [tmp] + [tmp] + [tmp])
    rate = rospy.Rate(10)  # 10hz
    for i in range(3):
        pub.publish(currentJointState)
        rospy.loginfo('Published!')
        rate.sleep()

    rospy.loginfo('end!')
