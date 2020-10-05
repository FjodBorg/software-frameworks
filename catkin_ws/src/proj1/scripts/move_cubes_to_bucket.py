#!/usr/bin/env python
import os
import rospkg
import rospy
import tf
import random
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *

import moveit_commander
import moveit_msgs.msg

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetWorldProperties


def gripper_close():
    return 0


def gripper_open():
    return 0


def find_cube(models, model_coordinates, p, scene):
    # TODO fix example code

    model_names = [i for i in models().model_names if "cube" in i]

    print(model_names)
    height = 1.4
    # scene.world.collision_objects.clear()
    cube_poses = []
    for model_name in model_names:
        cube_poses.append(model_coordinates(model_name, "").pose.position)
        # p.header.frame_id = robot.get_planning_frame()
        # p.pose.position = model_coordinates(model_name,"").pose.position
        # p.pose.position.z = height/2
        # scene.add_box(model_name, p, (0.5, 0.5, height))
    rospy.loginfo(cube_poses)
    # to access a position do cube_poses[0].x
    return cube_poses


def find_bucket(models, model_coordinates, p, scene):
    # Find the position of the bucket

    # Search of the object "bucket" in the model_states topic
    model_names = [i for i in models().model_names if "bucket" in i]
    
    bucket_pose = []
    for model_name in model_names:
        bucket_pose.append(model_coordinates(model_name, "").pose.position)

    print(bucket_pose)
    rospy.loginfo(bucket_pose)

    return bucket_pose


def move_path(models, model_coordinates, p, scene, group, robot):
    return 0


if __name__ == "__main__":

    rospy.init_node("move_cubes")
    models = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    model_coordinates = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("Arm")
    scene = moveit_commander.PlanningSceneInterface()
    p = geometry_msgs.msg.PoseStamped()

    find_cube(models, model_coordinates, p, scene)
    find_bucket(models, model_coordinates, p, scene)
    move_path(models, model_coordinates, p, scene, group, robot)
    gripper_open()
    gripper_close()
