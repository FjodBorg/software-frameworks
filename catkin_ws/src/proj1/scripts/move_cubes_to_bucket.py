#!/usr/bin/env python
import math
import sys
import rospy
import tf
import random
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel

# from geometry_msgs.msg import Point, Pose, Quaternion
import geometry_msgs

import moveit_commander
import moveit_msgs.msg

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetWorldProperties


def gripper_close():
    return 0


def gripper_open():
    return 0


def find_cube(models, model_coordinates, p, scene):
    # find all models with cube in them
    model_names = [i for i in models().model_names if "cube" in i]

    # height = 1.4
    # remove all objects from scene (does not work)
    # scene.world.collision_objects.clear()
    cube_poses = []
    for model_name in model_names:
        # extract all positions
        cube_poses.append(model_coordinates(model_name, "").pose.position)
        # p.header.frame_id = robot.get_planning_frame()
        # p.pose.position = model_coordinates(model_name,"").pose.position
        # p.pose.position.z = height/2
        # scene.add_box(model_name, p, (0.5, 0.5, height))
    rospy.loginfo(cube_poses)
    # to access a position do cube_poses[0].x
    return cube_poses


def find_bucket(models, model_coordinates, p, scene):

    # TODO fix example code

    # model_names = [i for i in models().model_names if "bucket" in i]

    # height = 1.4
    # scene.world.collision_objects.clear()
    # for model_name in model_names:
    # rospy.loginfo(model_coordinates(model_name,"").pose.position)
    # p.header.frame_id = robot.get_planning_frame()
    # p.pose.position = model_coordinates(model_name,"").pose.position
    # p.pose.position.z = height/2
    # scene.add_box(model_name, p, (0.5, 0.5, height))

    bucket_pos = 0
    return bucket_pos


# def move_path(models, model_coordinates, p, scene, group, robot):
def move_path(group, goal_pose, display_trajectory_publisher):
    curr_pose = group.get_current_pose().pose
    waypoints = []
    waypoints.append(curr_pose)
    goal_pose.orientation = geometry_msgs.msg.Quaternion(
        *tf_conversions.transformations.quaternion_from_euler(0.0, -math.pi / 2, 0.0)
    )
    waypoints.append(goal_pose)

    (plan1, _) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
    # waiting for RViz to display path
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory)
    rospy.sleep(1.0)

    # Moving to a pose goal
    group.execute(plan1, wait=True)
    rospy.sleep(5.0)

    return goal_pose


if __name__ == "__main__":
    rospy.init_node("move_cubes")
    models = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    model_coordinates = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    # initializing moveit related things
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("Arm")
    scene = moveit_commander.PlanningSceneInterface()

    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=1,
    )

    group.set_goal_orientation_tolerance(0.01)
    group.set_goal_tolerance(0.01)
    group.set_goal_joint_tolerance(0.01)
    group.set_planning_time(360)

    p = geometry_msgs.msg.PoseStamped()

    cube_points = find_cube(models, model_coordinates, p, scene)
    bucket_point = find_bucket(models, model_coordinates, p, scene)

    pose_goal = group.get_current_pose().pose
    pose_goal.position = cube_points[0]
    move_path(group, pose_goal, display_trajectory_publisher)
    # move_path(models, model_coordinates, p, scene, group, robot)

    gripper_open()
    gripper_close()
