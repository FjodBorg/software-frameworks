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
#from moveit_ros_planning_interface import _moveit_planning_scene_interface

 
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetWorldProperties


def gripper_close():
    return 0

def gripper_open():
    return 0

def find_cube(models, model_coordinates, p, scene):
    # TODO fix example code
    
    #model_names = [i for i in models().model_names if "cube" in i]
    
    #print(model_names)
    #height = 1.4
    #scene.world.collision_objects.clear()
    #for model_name in model_names:
        #rospy.loginfo(model_coordinates(model_name,"").pose.position)
        #p.header.frame_id = robot.get_planning_frame()
        #p.pose.position = model_coordinates(model_name,"").pose.position
        #p.pose.position.z = height/2
        #scene.add_box(model_name, p, (0.5, 0.5, height))
        
    cube_pos = 0
    return cube_pos

def find_bucket(models, model_coordinates, p, scene):

    # TODO fix example code

    #model_names = [i for i in models().model_names if "bucket" in i]
    
    #height = 1.4
    #scene.world.collision_objects.clear()
    #for model_name in model_names:
        #rospy.loginfo(model_coordinates(model_name,"").pose.position)
        #p.header.frame_id = robot.get_planning_frame()
        #p.pose.position = model_coordinates(model_name,"").pose.position
        #p.pose.position.z = height/2
        #scene.add_box(model_name, p, (0.5, 0.5, height))

    bucket_pos = 0
    return bucket_pos

def move_path(models, model_coordinates, p, scene, group, robot):
    return 0

if __name__ == "__main__":
    
    rospy.init_node("move_cubes")
    models = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
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
    
