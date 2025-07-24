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
    # move_to_home(move_group)

    # move_to_pose(move_group, -0.4777104651342358, 1.359001330710294, 1.2490419274547642, -1.929557662428473, 2.737636425207723, 2.4960563020971085, -0.20772273817410072)
    # grasp(grasp_client, 0.02)

    # move_to_pose(move_group, -1.0558913595885562, 1.4097066971242216, 1.2436439030295925, -1.6760797182627782, 2.737434869319199, 2.4917708044319355, -0.1960802372164447, 0.03938620910048485, 0.03938620910048485, 0.6)
    # grasp(grasp_client, 0.05)

    # move_to_home(move_group)

    move_to_home(move_group)
    move_to_pose(move_group, -0.505050320742423, -1.6532131750876442, 1.7703606261203162, -2.2126660369571907, 0.057320122092962264, 2.7077054580979873, -0.6423482887413765, 0.039349764585494995, 0.039349764585494995)
    grasp(grasp_client, 0.035)
    print("Press ENTER to grasp bowl ...")
    input()
    grasp(grasp_client, 0.03)
    move_to_pose(move_group, -1.9553351520631692, -1.7289233916600544, 1.764272976147765, -2.25836604827747, 0.15970306820339625, 3.5636313774663098, -0.781281786011325, 0.039349764585494995, 0.039349764585494995)
    move_to_pose(move_group, -2.3174051406017306, -1.7464846269616918, 1.7174293490793224, -1.8560868397825845, 0.19099589394198524, 3.5632729483577936, -0.8292145864632393, 0.03934943675994873, 0.03934943675994873)
    move_to_pose(move_group, -1.5547487497664334, -1.7548655496061893, 1.8098866582403617, -2.195141738907591, 0.19108510024017758, 3.6259514539374242, -0.7562993065979746, 0.03935009241104126, 0.03935009241104126)
    move_to_pose(move_group, -1.979525540786877, -1.7568802659049967, 1.7494914613459918, -1.6939346870251333, 0.1910150717364417, 3.625722295510163, -0.8387378708985116, 0.03934943675994873, 0.03934943675994873)
    move_to_pose(move_group, 0.04641734510079024, -1.7589041228378026, 1.791764214141974, -2.195651907233565, 0.19076389835940466, 3.096256562241962, -0.7286347739365364, 0.03935009241104126, 0.03935009241104126)
    move_to_pose(move_group, -0.37416483700484554, -1.7609308158640273, 1.6337540907943455, -1.4945198083342168, 0.1907236667373894, 3.478999657723638, -0.8049308530953194, 0.03934943675994873, 0.03934943675994873)
    move_to_pose(move_group, -0.3502987653160582, -1.7509824656574864, 1.623425122768145, -1.5075764313915316, 0.19090675082471636, 3.4782858388556375, 0.3781939363016022, 0.039350420236587524, 0.039350420236587524)
    move_to_pose(move_group, -0.37416483700484554, -1.7609308158640273, 1.6337540907943455, -1.4945198083342168, 0.1907236667373894, 3.478999657723638, -0.8049308530953194, 0.03934943675994873, 0.03934943675994873)
    move_to_pose(move_group, 0.04641734510079024, -1.7589041228378026, 1.791764214141974, -2.195651907233565, 0.19076389835940466, 3.096256562241962, -0.7286347739365364, 0.03935009241104126, 0.03935009241104126)
    move_to_pose(move_group, -0.505050320742423, -1.6532131750876442, 1.7703606261203162, -2.2126660369571907, 0.057320122092962264, 2.7077054580979873, -0.6423482887413765, 0.039349764585494995, 0.039349764585494995)
    print("Press ENTER to release bowl ...")
    input()
    grasp(grasp_client, 0.035)
    print("Press ENTER to when you removed the bowl ...")
    input()
    move_to_home(move_group)

    rospy.loginfo("Motion completed.")

    # Shutdown
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
