# coding=utf-8
# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.


# !/usr/bin/env python
# license removed for brevity

from threading import Thread
import os
import random
import time
import rosbag_record
import rospy
from gazebo_ros import gazebo_interface
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.msg import ModelStates, ModelState
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point


def movebase_client(pose):
    # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # Move 0.5 meters forward along the x axis of the "map" coordinate frame
    # 9.7 3.5
    x, y, w = pose
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = w

    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        # Result of executing the action
        return client.get_result()

    # If the python node is executed as main process (sourced directly)

def add_unmapped_objects(pose_lst= [[4,2,0], [0,1,0], [3,1,0], [-1,0,0], [0,-0.2,0], [1,0,0], [5,2.5,0], [5.5,3,0], [-2,1,0]]):

    pose_rand_lst = random.sample(pose_lst, 5)
    for i, pose in enumerate(pose_rand_lst):
        object_pose = Pose()
        object_pose.position.x = pose[0]
        object_pose.position.y = pose[1]
        object_pose.position.z = pose[2]

        print("------------ adding object "+str(i)+" to the map -------------")
        object_sdf = open('models/small_unit_cylinder/model.sdf').read()
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete_model(model_name="object"+str(i))
        gazebo_interface.spawn_sdf_model_client(model_name="object"+str(i), model_xml=object_sdf,
                                                robot_namespace=rospy.get_namespace(), initial_pose=object_pose,
                                                reference_frame="", gazebo_namespace="/gazebo")

if __name__ == '__main__':


    for i in range(5):
        # add_unmapped_objects()
        thread = Thread(target=os.system, args=["python velocity_attack.py"])
        thread.start()

        # os.system("python velocity_attack.py")
        # start rosbag recording
        cmd, rosbag_proc = rosbag_record.start_record("rosbag record -a -b 1 -o bag_records/abnormal/vel_atk")
        print("------------ start recording -------------")
        time.sleep(5)
        try:

            # Initializes a rospy node to let the SimpleActionClient publish and subscribe

            print("\n------------ navigate to goal ------------ ")
            rospy.init_node('movebase_client_py')
            # result = movebase_client([-2, 2, 1])
            # if result:
            #     rospy.loginfo("Goal execution done!")
            # Initializes a rospy node to let the SimpleActionClient publish and subscribe
            # print("second goal")
            result = movebase_client([9, 2.5, 1])
            if result:
                rospy.loginfo("Goal execution done!")
            # Initializes a rospy node to let the SimpleActionClient publish and subscribe
            print("------------ back to start position ------------")
            result = movebase_client([0, 0, 1])
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

        print("------------ end recording -------------")
        rosbag_record.end_record(cmd, rosbag_proc)
        thread.join()
        time.sleep(2)
