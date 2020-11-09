#!/usr/bin/env python
import copy
import math
import sys
import rospy
import tf
import random
import tf_conversions
from gazebo_msgs.srv import DeleteModel, SpawnModel
from sensor_msgs.msg import JointState

# from geometry_msgs.msg import Point, Pose, Quaternion
import geometry_msgs

import moveit_commander
import moveit_msgs.msg

from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetWorldProperties


def jointStatesCallback(msg):
    global currentJointState
    currentJointState = msg


def gripper_close():
    # Setup subscriber
    # rospy.Subscriber("/joint_states", JointState, jointStatesCallback)

    pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)

    currentJointState = rospy.wait_for_message("/joint_states", JointState)
    rospy.loginfo("Received!")
    currentJointState.header.stamp = rospy.get_rostime()
    tmp = 0.75
    # tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
    currentJointState.position = tuple(
        list(currentJointState.position[:6]) + [tmp] + [tmp] + [tmp]
    )
    rate = rospy.Rate(10)  # 10hz
    for i in range(3):
        pub.publish(currentJointState)
        rospy.loginfo("Published!")
        rate.sleep()

    return 0


def gripper_open():
    # Setup subscriber
    # rospy.Subscriber("/joint_states", JointState, jointStatesCallback)

    pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)

    currentJointState = rospy.wait_for_message("/joint_states", JointState)
    rospy.loginfo("Received!")
    currentJointState.header.stamp = rospy.get_rostime()
    tmp = 0.005
    # tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
    currentJointState.position = tuple(
        list(currentJointState.position[:6]) + [tmp] + [tmp] + [tmp]
    )
    rate = rospy.Rate(10)  # 10hz
    for i in range(3):
        pub.publish(currentJointState)
        rospy.loginfo("Published!")
        rate.sleep()

    return 0


def find_cube(models, model_coordinates, robot, scene):
    # find all models with cube in them
    model_names = [i for i in models().model_names if "cube" in i]

    # height = 1.4
    # remove all objects from scene (does not work)
    # scene.world.collision_objects.clear()
    cube_poses = []

    for model_name in model_names:
        # extract all positions
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = robot.get_planning_frame()
        p.pose = model_coordinates(model_name, "").pose
        p.pose.position.z += 0.17

        cube_poses.append(p)
        # scene.add_box(model_name, p, (0.05, 0.05, 0.05))
        # rospy.loginfo(model_name)
        # rospy.loginfo(p)
    rospy.loginfo(cube_poses)
    # to access a position do cube_poses[0].x
    return cube_poses


def find_bucket(models, model_coordinates, robot, scene):
    # Find the position of the bucket

    # Search of the object "bucket" in the model_states topic
    model_names = [i for i in models().model_names if "bucket" in i]
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose = model_coordinates(model_names[0], "").pose
    p.pose.position.z += 0.09
    scene.add_box(model_names[0], p, (0.22, 0.22, 0.20))

    return p.pose


def add_table_scene(robot, scene):
    # Find the position of the bucket

    # Search of the object "bucket" in the model_states topic
    p = geometry_msgs.msg.PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0.683
    scene.add_box("table", p, (2, 2, 0.10))

    rospy.loginfo(p)

    return p.pose


# def move_path(models, model_coordinates, p, scene, group, robot):
def move_path(
    group, goal_pose, display_trajectory_publisher, vert_approach=True, vert_offset=0.25
):
    rospy.sleep(0.2)
    curr_pose = group.get_current_pose().pose
    rospy.loginfo("Initial pose...\n{}".format(curr_pose.position))

    waypoints = []
    waypoints.append(curr_pose)

    curr_pose.orientation = goal_pose.orientation
    waypoints.append(curr_pose)
    move_a2b(group, waypoints, display_trajectory_publisher)

    # movin to vert_offset m above
    waypoints = []
    curr_pose = group.get_current_pose().pose
    waypoints.append(curr_pose)
    inter_pose = copy.deepcopy(goal_pose)
    inter_pose.position.z += vert_offset
    waypoints.append(inter_pose)
    move_a2b(group, waypoints, display_trajectory_publisher)

    # dipping down for cubes
    if vert_approach:
        waypoints = []
        curr_pose = group.get_current_pose().pose
        waypoints.append(curr_pose)
        waypoints.append(goal_pose)
        move_a2b(group, waypoints, display_trajectory_publisher)

        gripper_close()
        rospy.sleep(0.5)

        # resetting above to avoid collisions
        waypoints = []
        curr_pose = group.get_current_pose().pose
        waypoints.append(curr_pose)
        waypoints.append(inter_pose)
        move_a2b(group, waypoints, display_trajectory_publisher)
    else:
        gripper_open()
        rospy.sleep(0.5)

    # printing final position
    rospy.loginfo(
        "Final position...\n{}".format(group.get_current_pose().pose.position)
    )


def move_a2b(group, waypoints, display_trajectory_publisher):
    rospy.loginfo(
        "Moving from...\n{}\nto...\n{}".format(
            waypoints[0].position, waypoints[-1].position
        )
    )
    (plan1, _) = group.compute_cartesian_path(waypoints, 0.01, 0.0)
    # rospy.sleep(2.0)
    # waiting for RViz to display path
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan1)
    display_trajectory_publisher.publish(display_trajectory)
    rospy.sleep(1.0)

    # Moving to a pose goal
    group.execute(plan1, wait=True)
    rospy.sleep(0.5)


if __name__ == "__main__":
    rospy.init_node("move_cubes")
    models = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    model_coordinates = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

    # initializing moveit related things
    moveit_commander.roscpp_initialize(sys.argv)
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("Arm")

    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=1e3,
    )
    rospy.sleep(2)

    group.set_goal_orientation_tolerance(0.01)
    group.set_goal_tolerance(0.01)
    group.set_goal_joint_tolerance(0.01)
    group.set_planning_time(1e3)

    p = geometry_msgs.msg.PoseStamped()

    # add_table_scene(robot, scene)
    pose_cubes = find_cube(models, model_coordinates, robot, scene)
    pose_bucket = find_bucket(models, model_coordinates, robot, scene)

    # motion commands for a single cube-pickup and dropoff
    # should repeat for cube in pose_cubes
    pose_goal = group.get_current_pose().pose
    pose_goal.orientation = geometry_msgs.msg.Quaternion(
        *tf_conversions.transformations.quaternion_from_euler(
            0.0, -math.pi / 2, -math.pi / 4
        )
    )

    gripper_open()
    for cube in pose_cubes:
        # goto cube
        rospy.loginfo("Going to cube...")
        pose_goal.position = copy.deepcopy(cube.pose.position)
        move_path(
            group,
            pose_goal,
            display_trajectory_publisher,
            vert_approach=True,
            vert_offset=0.5,
        )
        # goto bucket
        rospy.loginfo("Going to bucket...")
        pose_goal.position = copy.deepcopy(pose_bucket.position)
        move_path(
            group,
            pose_goal,
            display_trajectory_publisher,
            vert_approach=False,
            vert_offset=0.5,
        )

