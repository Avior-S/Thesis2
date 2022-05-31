#!/usr/bin/env python

# import sys
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# from geometry_msgs.msg import Pose, PoseStamped
# import numpy as np
# import genpy
# from gazebo_ros import gazebo_interface
# from gazebo_msgs.srv import DeleteModel
# from gazebo_msgs.srv import DeleteModel, ApplyJointEffort, ApplyJointEffortRequest, JointRequest
# from sensor_msgs.msg import JointState
# from keyboard_subscriber import keyboard_subscribers
# from gazebo_msgs.msg import ModelStates, ModelState
from math import radians, pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from gripper_controller import *
from panda_functions import *
import time
import copy
import rosbag_record



def move_group_python_interface(controllers=None):

    initialize_environment2()

    # initialize all real and simulation controllers
    if not controllers:
        controllers = All_Controllers()
        stop_monitor_in_the_end = True
    else:
        stop_monitor_in_the_end = False

    # Step 0: Start at "Home" pos w\ open gripper
    panda_joints = rospy.wait_for_message("/joint_states", JointState)
    arm_joints = np.around(panda_joints.position[2:], 3)
    # if not in home -> goto home
    if arm_joints[3] != -0.07:
        print('Going to Home Position')
        controllers.group.set_joint_value_target([0.0, 0.0, 0.0, -0.07, 0.0, 0.0, 0.7853])
        plan = controllers.group.plan()
        if plan.joint_trajectory.joint_names != []:
            plan = trajectory_speed_up(plan, 4)
            controllers.group.execute(plan, wait=False)
            panda_wait_for_stop()
    hand_joints = np.around(panda_joints.position[:2], 3)
    # if hand not open -> open hand
    if hand_joints[0] != 0.04:
        gripper_open(gripper=controllers.sim_gripper)

    # Step 1 - add an object
    print("------------ step 1 -------------")
    object_sdf = open('model.sdf').read()
    # open('/home/dan/franka/franka_ros/src/franka_python/models/apple.sdf').read()

    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
    model_states = rospy.wait_for_message("gazebo/model_states", ModelStates)
    # make sure the object is not in the simulation
    if "object1" in model_states.name:
        # # print("condition true")
        # print(model_states.name)
        delete_model(model_name="object1")
    # print("end condition")
    gazebo_interface.spawn_sdf_model_client(model_name="object1", model_xml=object_sdf,
                                            robot_namespace=rospy.get_namespace(), initial_pose=object_pose,
                                            reference_frame="", gazebo_namespace="/gazebo")


    # start rosbag recording
    print("------------ step 1.5 -------------")
    cmd, rosbag_proc = rosbag_record.start_record("rosbag record -a -b 1 -o bag_records/panda_test")#rosbag record -a -b 1 -o ./bag_records/panda_sim")
    time.sleep(5)

    # Step 2 - pre-grasp position
    print("------------ step 2 -------------")
    controllers.group.set_pose_target(pose1)
    plan1 = controllers.group.plan()
    if plan1.joint_trajectory.joint_names!=[]:
        controllers.group.execute(plan1, wait=False)
        panda_wait_for_stop()
    else:
        print("Manipulator's plan is not valid")
    # prepare the gripper (if gear2)
    #gripper_position(object_width=0.8, gripper=controllers.sim_gripper)

    # Step 2.5 - lower to grasp position
    print("------------ step 2.5 -------------")
    # try 10 times to find a lin motion by re-planning (probabilistic planner)
    a = 0
    panda_wait_for_stop()
    waypoints = []
    waypoints.append(copy.deepcopy(pose1))
    pose1.position.z -= 0.139
    waypoints.append(copy.deepcopy(pose1))
    while a < 10:
        (plan, fraction) = controllers.group.compute_cartesian_path(waypoints=waypoints, eef_step=0.005, jump_threshold=0)
        if fraction != 0 and plan.joint_trajectory.joint_names != [] and np.sum(np.abs(plan.joint_trajectory.points[-2].velocities)) > 0:
            if linear_trajectory_check(plan):  # Makes sure that the whole trajectory is in a linear movement
                plan = trajectory_speed_up(plan, speed=0.5)
                controllers.group.execute(plan, wait=False)
                panda_wait_for_stop()
                a += 10
                print ('executed lin. trajectory')

        a += 1
        print (a)

    # Step 3 - close the gripper
    print("------------ step 3 -------------")
    #raw_input('close-enter')
    gripper_close_on_object(object_width=0.025, gripper=controllers.sim_gripper) # 0.08 for apple

    # Step 4 - pick the object
    print("------------ step 4 -------------")
    controllers.group.set_pose_target(pose2)
    plan1 = controllers.group.plan()
    if plan1.joint_trajectory.joint_names!=[]:
        controllers.group.execute(plan1, wait=False)
        panda_wait_for_stop()
    else:
        print("Manipulator's plan is not valid")

    # # Print stuff
    # print ("get_joint_value_target", controllers.group.get_joint_value_target())
    # print ("get_current_pose", controllers.group.get_current_pose())
    # print ("get_active_joints", controllers.group.get_active_joints())
    # print ("get_current_joint_values", controllers.group.get_current_joint_values())
    # print ("get_joints", controllers.group.get_joints())

    # Step 5 - go to pose 2
    print("------------ step 5 -------------")
    controllers.group.set_pose_target(pose2)
    plan1 = controllers.group.plan()
    plan1 = trajectory_speed_up(plan1, speedfactor)
    controllers.group.execute(plan1, wait=False)
    panda_wait_for_stop()

    # step 6 - goto drop zone
    print("------------ step 6 -------------")
    pose3.position.z += 0.145
    controllers.group.set_pose_target(pose3)
    plan1 = controllers.group.plan()
    plan1 = trajectory_speed_up(plan1, speedfactor)
    controllers.group.execute(plan1, wait=False)
    panda_wait_for_stop()

    # Step 7 - drop the object
    print("------------ step 7 -------------")
    gripper_position(object_width=0.067, gripper=controllers.sim_gripper)
    #gripper_open(gripper=controllers.sim_gripper)

    # Step 7.5 - lin motion up
    print("------------ step 7.5 -------------")
    # try 10 times to find a lin motion by re-planning (probabilistic planner)
    a = 0
    panda_wait_for_stop()
    waypoints = []
    waypoints.append(copy.deepcopy(pose3))
    pose3.position.z += 0.145
    waypoints.append(copy.deepcopy(pose3))
    while a < 10:
        (plan, fraction) = controllers.group.compute_cartesian_path(waypoints=waypoints, eef_step=0.005, jump_threshold=0)
        if fraction != 0 and plan.joint_trajectory.joint_names != [] and np.sum(np.abs(plan.joint_trajectory.points[-2].velocities)) > 0:
            if linear_trajectory_check(plan):  # Makes sure that the whole trajectory is in a linear movement
                plan = trajectory_speed_up(plan, speed=0.5)
                controllers.group.execute(plan, wait=True)
                panda_wait_for_stop()
                a += 10
                print ('executed lin. trajectory')

        a += 1
        print (a)

    # Step 8 - go back 'home'
    print("------------ step 8 -------------")
    controllers.group.set_joint_value_target([0.0, 0.0, 0.0, -0.07, 0.0, 0.0, 0.7853])
    #controllers.group.set_named_target("home")
    controllers.group.go()
    print (' finish ')
    rosbag_record.end_record(cmd, rosbag_proc)

if __name__ == '__main__':
    speedfactor = 4
    n = 5

    for i in range(n):
        # do n pick and place actions
        # set predefined grasp poses
        # above the apple
        pose1 = Pose()
        pose1.position.x = 0.45
        pose1.position.y = 0.13
        pose1.position.z = 0.01 + 0.15 + 0.1025  # compensate: apple 0.07, gear 0.01 and for 'tcp to eef-link' distance in panda by +0.1025
        q = quaternion_from_euler(ai=radians(-180), aj=radians(0), ak=radians(-45))
        pose1.orientation.x = q[0]
        pose1.orientation.y = q[1]
        pose1.orientation.z = q[2]
        pose1.orientation.w = q[3]

        # above the middle of table
        pose2 = Pose()
        pose2.position.x = 0.0
        pose2.position.y = 0.4
        pose2.position.z = 0.4
        q = quaternion_from_euler(ai=radians(-180), aj=radians(0), ak=radians(-45))
        pose2.orientation.x = q[0]
        pose2.orientation.y = q[1]
        pose2.orientation.z = q[2]
        pose2.orientation.w = q[3]

        # at the drop zone
        pose3 = Pose()
        pose3.position.x = -0.45
        pose3.position.y = 0.13
        pose3.position.z = 0.01 + 0.1025 + 0.02  # apple +0.005/ gear +0.02 a little above the table to avoid obj-table collisions
        q = quaternion_from_euler(ai=radians(-180), aj=radians(0), ak=radians(-45))
        pose3.orientation.x = q[0]
        pose3.orientation.y = q[1]
        pose3.orientation.z = q[2]
        pose3.orientation.w = q[3]

        # object placement
        object_pose = Pose()
        object_pose.position.x = 0.45
        object_pose.position.y = 0.13
        object_pose.position.z = 0.011
        object_pose.orientation.x = 0
        object_pose.orientation.y = 0
        object_pose.orientation.z = 0.8
        object_pose.orientation.w = 1

        try:
            move_group_python_interface()
        except rospy.ROSInterruptException:
            pass
