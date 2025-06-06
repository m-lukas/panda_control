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

    move_to_pose(move_group, -0.09891566097543623, 0.6828904053704781, -0.23959539453397716, -1.6735154627749793, 0.17013589685374067, 2.372188758742326, 0.3660043845698237, 0.03938949480652809, 0.03938949480652809)
    move_to_pose(move_group, -0.10074728628940749, 0.7381568269046928, -0.2398823979424306, -1.671042809469658, 0.17013853591414344, 2.372645607717684, 0.3665108754361669, 0.03938949480652809, 0.03938949480652809)
    grasp(grasp_client, 0.06)
    move_to_pose(move_group, 0.3007464123719343, 0.628549533025106, -0.005302090632549503, -1.6508580494596248, 0.01620031431683505, 2.253569281604555, 1.0876608148265887, 0.03938916325569153, 0.03938916325569153)
    move_to_pose(move_group, 0.2910494713574125, 0.772953148488121, 0.00662456731304352, -1.5684667512216857, 0.023626947488541716, 2.2574722571591357, 1.0515771087667352, 0.03938949480652809, 0.03938949480652809)
    grasp(grasp_client, 0.08)
    move_to_pose(move_group, 0.2765186185256223, 0.6938366992214425, 0.014287254946165926, -1.5373419803119555, 0.023615427619905847, 2.267180141525654, 1.0512268295014897, 0.03938949480652809, 0.03938949480652809)
    move_to_home(move_group)

    rospy.loginfo("Motion completed.")

    # Shutdown
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
