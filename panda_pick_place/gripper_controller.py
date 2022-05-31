#!/usr/bin/env python

import sys
import genpy
import rospy
import moveit_commander
from moveit_msgs.msg import MoveGroupActionResult
from control_msgs.msg import FollowJointTrajectoryActionFeedback
from geometry_msgs.msg import Pose, PoseStamped
import time
import numpy as np
from math import pi, cos, acos
from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import DeleteModel, ApplyJointEffort, ApplyJointEffortRequest, JointRequest
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates, ModelStates
import threading
from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.msg import RobotTrajectory


def gripper_wait_for_stop():
    # Wait for the gripper to Stop
    time.sleep(0.1)
    controller_state = rospy.wait_for_message("/EffortJointInterface_trajectory_gripper/state", JointTrajectoryControllerState)
    while controller_state.desired.velocities!=(0,0):
        controller_state = rospy.wait_for_message("/EffortJointInterface_trajectory_gripper/state", JointTrajectoryControllerState)
        #time.sleep(0.1)
    return



def gripper_position(object_width = None, gripper = None):
    # Get the grippers fingers to a specific position
    if not gripper:
        gripper = moveit_commander.MoveGroupCommander("hand")
    if not object_width:
        object_width = 0.0
    clear_joint_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
    clear_joint_effort('panda_finger_joint1')
    clear_joint_effort('panda_finger_joint2')
    joint_goal = [object_width/2, object_width/2]
    gripper.set_joint_value_target(joint_goal)
    plan = gripper.plan()
    plan = gripper_speed_up(plan, speed=1)
    gripper.execute(plan, wait=True)
    #gripper.go(joint_goal, wait=True)
    #gripper.stop()
    #time.sleep(0.1)
    gripper_wait_for_stop()

def gripper_close_on_object(object_width=None, gripper=None, cleareffort=True, tighten=True, gear2=False):
    # Close the gripper on a object
    #raw_input("NOW, closing - press enter")
    if not gripper:
        gripper = moveit_commander.MoveGroupCommander('hand')
    if cleareffort:
        clear_joint_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
        clear_joint_effort('panda_finger_joint1')
        clear_joint_effort('panda_finger_joint2')
    # get position
    if not object_width:
        joint_goal = [0.04, 0.04]
    else:
        joint_goal = [object_width*0.5, object_width*0.5]
    # move the gripper
    gripper.set_joint_value_target(joint_goal)
    plan = gripper.plan()
    plan = gripper_speed_up(plan, speed=0.5)
    gripper.execute(plan, wait=True)
    #raw_input("NOW, tightening grip - press enter")
    if tighten:
        # tighten the grip with extra effort (on the object) - step1
        left_effort = 1 * -5
        right_effort = 1 * -5
        # Send to gazebo (service)
        joint_effort_left_finger = ApplyJointEffortRequest('panda_finger_joint1', left_effort, genpy.Time(),
                                                           genpy.Duration(-1))
        joint_effort_right_finger = ApplyJointEffortRequest('panda_finger_joint2', right_effort, genpy.Time(),
                                                            genpy.Duration(-1))
        apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        apply_joint_effort(joint_effort_left_finger)
        apply_joint_effort(joint_effort_right_finger)
        time.sleep(0.2)
        # tighten the grip with extra effort (on the object) - step2
        left_effort = 5 * -5
        right_effort = 5 * -5
        # Send to gazebo (service)
        joint_effort_left_finger = ApplyJointEffortRequest('panda_finger_joint1', left_effort, genpy.Time(),
                                                           genpy.Duration(-1))
        joint_effort_right_finger = ApplyJointEffortRequest('panda_finger_joint2', right_effort, genpy.Time(),
                                                            genpy.Duration(-1))
        apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        apply_joint_effort(joint_effort_left_finger)
        apply_joint_effort(joint_effort_right_finger)
        time.sleep(0.2)
    #raw_input('liftoff - enter')

    # if tighten:  # tighten the grip linearly to full effort 10 - 100%
    #     tt = 1
    #     while tt < 11:
    #         # tighten the grip with extra effort (on the object)
    #         left_effort = tt * -2
    #         right_effort = tt * -2
    #         # Send to gazebo (service)
    #         joint_effort_left_finger = ApplyJointEffortRequest('panda_finger_joint1', left_effort, genpy.Time(),
    #                                                            genpy.Duration(-1))
    #         joint_effort_right_finger = ApplyJointEffortRequest('panda_finger_joint2', right_effort, genpy.Time(),
    #                                                             genpy.Duration(-1))
    #         apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
    #         apply_joint_effort(joint_effort_left_finger)
    #         apply_joint_effort(joint_effort_right_finger)
    #         time.sleep(0.05)
    #         tt += 1
    #     time.sleep(0.2)



def gripper_open(joint_goal=None, gripper=None):
    # Get the grippers fingers to the open position
    if joint_goal is None:
        joint_goal = [0.04, 0.04]
    if not gripper:
        gripper = moveit_commander.MoveGroupCommander("hand")
    clear_joint_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
    clear_joint_effort('panda_finger_joint1')
    clear_joint_effort('panda_finger_joint2')
    #print('Moving Fingers to position [Left,Right]:',joint_goal)
    gripper.set_joint_value_target(joint_goal)
    plan = gripper.plan()
    plan = gripper_speed_up(plan, speed=1)
    gripper.execute(plan, wait=True)
    #gripper.go(joint_goal, wait=True)
    #gripper.stop()
    time.sleep(0.1)
    #gripper_wait_for_stop()

def gripper_close(joint_goal=None, gripper=None, tighten = False, cleareffort=True):
    # Get the grippers fingers to the closed position
    if joint_goal is None:
        joint_goal = [0.0, 0.0]
    if not gripper:
        gripper = moveit_commander.MoveGroupCommander("hand")
    if cleareffort == True:
        clear_joint_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
        clear_joint_effort('panda_finger_joint1')
        clear_joint_effort('panda_finger_joint2')
    #print('Moving Fingers to position [Left,Right]:',joint_goal)
    gripper.set_joint_value_target(joint_goal)
    plan = gripper.plan()
    plan = gripper_speed_up(plan, speed=1)
    gripper.execute(plan, wait=True)
    #gripper.go(joint_goal, wait=True)
    #gripper.stop()
    #time.sleep(0.1)
    if tighten == True:
        # tighten the grip with extra effort (on the object)
        left_effort = -5 * 20
        right_effort = -5 * 20
        #time.sleep(0.1)
        # Send to gazebo (service)
        joint_effort_left_finger = ApplyJointEffortRequest('panda_finger_joint1', left_effort, genpy.Time(),
                                                           genpy.Duration(-1))
        joint_effort_right_finger = ApplyJointEffortRequest('panda_finger_joint2', right_effort, genpy.Time(),
                                                            genpy.Duration(-1))
        apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        apply_joint_effort(joint_effort_left_finger)
        apply_joint_effort(joint_effort_right_finger)
        time.sleep(0.1)



def gripper_speed_up(trajectory, speed=1.0):
    new_trajectory = RobotTrajectory()
    new_trajectory.joint_trajectory = trajectory.joint_trajectory
    n_points = len(trajectory.joint_trajectory.points)
    for i in range(n_points):
        new_trajectory.joint_trajectory.points[i].time_from_start = trajectory.joint_trajectory.points[i].time_from_start / speed
        velocities = (new_trajectory.joint_trajectory.points[i].velocities[0] * speed, new_trajectory.joint_trajectory.points[i].velocities[1] * speed)
        accelerations = (new_trajectory.joint_trajectory.points[i].accelerations[0] * speed, new_trajectory.joint_trajectory.points[i].accelerations[1] * speed)
        new_trajectory.joint_trajectory.points[i].velocities = velocities
        new_trajectory.joint_trajectory.points[i].accelerations = accelerations
    return new_trajectory



if __name__ == "__main__":
    gripper_close()
    gripper_open()






