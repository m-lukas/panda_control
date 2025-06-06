#!/usr/bin/env python3

import rospy
import moveit_commander
import geometry_msgs.msg
from math import pi
import sys
import actionlib
from franka_gripper.msg import HomingAction, HomingGoal, MoveAction, MoveGoal
from controls import home_gripper, grasp, move_to_home, move_to_pose


def control():
    # Initialize moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('panda_move_arm_node', anonymous=True)

    home_gripper_client = actionlib.SimpleActionClient('/franka_gripper/homing', HomingAction)
    home_gripper_client.wait_for_server()

    grasp_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
    grasp_client.wait_for_server()

    # Instantiate a RobotCommander object (interface to the robot)
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object (interface to the world)
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate MoveGroupCommander for the arm
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Print robot info
    rospy.loginfo("Planning frame: %s" % move_group.get_planning_frame())
    rospy.loginfo("End effector link: %s" % move_group.get_end_effector_link())
    rospy.loginfo("Available Planning Groups: %s" % robot.get_group_names())

    home_gripper(home_gripper_client)
    move_to_home(move_group)

    move_to_pose(move_group, 0.007502125432747498, -0.4595430795853597, -0.34150164016457585, -2.344187818994454, -0.14758739797496137, 1.9206552899943456, 0.5975519083067775, 0.0393441841006279, 0.0393441841006279)
    move_to_pose(move_group, -0.010257551462144781, 0.24661863172054288, -0.3731552714784219, -1.9782862786633284, 0.21283133847072816, 2.2662716703679826, 0.5910133928817104, 0.039344511926174164, 0.039344511926174164)
    move_to_pose(move_group, -0.035248506227606224, 0.9137405539897451, -0.1455794884154671, -1.1604185058677392, 0.7711146025929062, 2.4611001870095706, 0.9079143599044697, 0.039344511926174164, 0.039344511926174164)
    move_to_pose(move_group, 0.04218337227899637, 0.912616796527936, 0.6135963478088372, -1.031564836147913, 0.7723889816900094, 2.4620866228500358, 1.636818359201646, 0.039344511926174164, 0.039344511926174164)
    move_to_pose(move_group, -0.26260307709218256, 0.49043448354185026, 0.641169696209106, -0.9843506892356332, 0.7717479663193225, 2.4579625942628645, 1.633392367387446, 0.039344511926174164, 0.039344511926174164)
    move_to_pose(move_group, 0.03970028055446189, -0.708539665565156, 0.732246931567036, -2.4791089605057977, 0.7700795706312248, 2.4583154516484997, 1.3216107776832218, 0.0393441841006279, 0.0393441841006279)

    move_to_home(move_group)

    rospy.loginfo("Motion completed.")

    # Shutdown
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
