from typing import Callable, Dict, List

from controls import DEFAULT_ARM_SPEED, move_to_pose, home_gripper, move_to_home, grasp
from notifier import notify_arm_location


class MoveToHandoverProgram:
    def __init__(self, target_1: tuple, target_2: tuple, speed: float):
      self.target_1 = target_1
      self.target_2 = target_2
      self.speed = speed


class ContainerProgram:
    def __init__(self, container_pose: tuple, rotation_pose: tuple):
      self.container_pose = container_pose
      self.rotation_pose = rotation_pose


left_tray_index = 0
right_tray_index = 0

packaging_containers: List[ContainerProgram] = [
    ContainerProgram(
        [-0.37416483700484554, -1.7609308158640273, 1.6337540907943455, -1.4945198083342168, 0.1907236667373894, 3.478999657723638, -0.8049308530953194, 0.03934943675994873, 0.03934943675994873],
        [-0.3502987653160582, -1.7509824656574864, 1.623425122768145, -1.5075764313915316, 0.19090675082471636, 3.4782858388556375, 0.3781939363016022, 0.039350420236587524, 0.039350420236587524]
    ),
    ContainerProgram(
        [-0.37416483700484554, -1.7609308158640273, 1.6337540907943455, -1.4945198083342168, 0.1907236667373894, 3.478999657723638, -0.8049308530953194, 0.03934943675994873, 0.03934943675994873],
        [-0.3502987653160582, -1.7509824656574864, 1.623425122768145, -1.5075764313915316, 0.19090675082471636, 3.4782858388556375, 0.3781939363016022, 0.039350420236587524, 0.039350420236587524]
    ),
    ContainerProgram(
        [-0.37416483700484554, -1.7609308158640273, 1.6337540907943455, -1.4945198083342168, 0.1907236667373894, 3.478999657723638, -0.8049308530953194, 0.03934943675994873, 0.03934943675994873],
        [-0.3502987653160582, -1.7509824656574864, 1.623425122768145, -1.5075764313915316, 0.19090675082471636, 3.4782858388556375, 0.3781939363016022, 0.039350420236587524, 0.039350420236587524]
    ),
    ContainerProgram(
        [-0.37416483700484554, -1.7609308158640273, 1.6337540907943455, -1.4945198083342168, 0.1907236667373894, 3.478999657723638, -0.8049308530953194, 0.03934943675994873, 0.03934943675994873],
        [-0.3502987653160582, -1.7509824656574864, 1.623425122768145, -1.5075764313915316, 0.19090675082471636, 3.4782858388556375, 0.3781939363016022, 0.039350420236587524, 0.039350420236587524]
    ),
    ContainerProgram(
        [-0.37416483700484554, -1.7609308158640273, 1.6337540907943455, -1.4945198083342168, 0.1907236667373894, 3.478999657723638, -0.8049308530953194, 0.03934943675994873, 0.03934943675994873],
        [-0.3502987653160582, -1.7509824656574864, 1.623425122768145, -1.5075764313915316, 0.19090675082471636, 3.4782858388556375, 0.3781939363016022, 0.039350420236587524, 0.039350420236587524]
    ),
]
current_container_index = 0
number_of_items_in_container = 0


left_tray_handovers: List[MoveToHandoverProgram] = [
    MoveToHandoverProgram(
        [-1.9553351520631692, -1.7289233916600544, 1.764272976147765, -2.25836604827747, 0.15970306820339625, 3.5636313774663098, -0.781281786011325, 0.039349764585494995, 0.039349764585494995],
        [-2.3174051406017306, -1.7464846269616918, 1.7174293490793224, -1.8560868397825845, 0.19099589394198524, 3.5632729483577936, -0.8292145864632393, 0.03934943675994873, 0.03934943675994873],
        DEFAULT_ARM_SPEED
    )
]
right_tray_handovers: List[MoveToHandoverProgram] = [
    MoveToHandoverProgram(
        [-1.5547487497664334, -1.7548655496061893, 1.8098866582403617, -2.195141738907591, 0.19108510024017758, 3.6259514539374242, -0.7562993065979746, 0.03935009241104126, 0.03935009241104126],
        [-1.979525540786877, -1.7568802659049967, 1.7494914613459918, -1.6939346870251333, 0.1910150717364417, 3.625722295510163, -0.8387378708985116, 0.03934943675994873, 0.03934943675994873],
        DEFAULT_ARM_SPEED
    )
]


def move_to_left_tray(move_group):
    program = left_tray_handovers[left_tray_index]

    # move to close to handover location
    move_to_pose(move_group, program.target_1[0], program.target_1[1], program.target_1[2], program.target_1[3], program.target_1[4], program.target_1[5], program.target_1[6], program.target_1[7], None, None, program.speed)
    notify_arm_location("handover_location")

    # move to specific handover location
    move_to_pose(move_group, program.target_2[0], program.target_2[1], program.target_2[2], program.target_2[3], program.target_2[4], program.target_2[5], program.target_2[6], program.target_2[7])
    left_tray_handovers += 1


def move_to_right_tray(move_group):
    program = right_tray_handovers[right_tray_index]

    # move to close to handover location
    move_to_pose(move_group, program.target_1[0], program.target_1[1], program.target_1[2], program.target_1[3], program.target_1[4], program.target_1[5], program.target_1[6], program.target_1[7], None, None, program.speed)
    notify_arm_location("handover_location")

    # move to specific handover location
    move_to_pose(move_group, program.target_2[0], program.target_2[1], program.target_2[2], program.target_2[3], program.target_2[4], program.target_2[5], program.target_2[6], program.target_2[7])
    right_tray_handovers += 1


def move_to_packaging(move_group, speed=DEFAULT_ARM_SPEED):
    # move to packaging common
    move_to_pose(move_group, 0.04641734510079024, -1.7589041228378026, 1.791764214141974, -2.195651907233565, 0.19076389835940466, 3.096256562241962, -0.7286347739365364, None, None, speed)
    notify_arm_location("packaging")

    # move to specific container and rotate
    cp = packaging_containers[current_container_index]
    move_to_pose(move_group, cp.container_pose[0], cp.container_pose[1], cp.container_pose[2], cp.container_pose[3], cp.container_pose[4], cp.container_pose[5], cp.container_pose[6], cp.container_pose[7], None, None, speed)
    move_to_pose(move_group, cp.rotation_pose[0], cp.rotation_pose[1], cp.rotation_pose[2], cp.rotation_pose[3], cp.rotation_pose[4], cp.rotation_pose[5], cp.rotation_pose[6], cp.rotation_pose[7], None, None, speed)
    move_to_pose(move_group, cp.container_pose[0], cp.container_pose[1], cp.container_pose[2], cp.container_pose[3], cp.container_pose[4], cp.container_pose[5], cp.container_pose[6], cp.container_pose[7], None, None, speed)

    number_of_items_in_container += 1
    if number_of_items_in_container == 8:
        current_container_index += 1
        number_of_items_in_container = 0
        # account for errors
    
    # move to packaging common
    move_to_pose(move_group, 0.04641734510079024, -1.7589041228378026, 1.791764214141974, -2.195651907233565, 0.19076389835940466, 3.096256562241962, -0.7286347739365364, None, None, speed)
    notify_arm_location("handover_finished")


def move_to_idle(move_group, speed=DEFAULT_ARM_SPEED):
    move_to_pose(move_group, -0.505050320742423, -1.6532131750876442, 1.7703606261203162, -2.2126660369571907, 0.057320122092962264, 2.7077054580979873, -0.6423482887413765, None, None, speed)
    notify_arm_location("idle")


PROGRAMS: Dict[str, Callable] = {
    "move_to_left_tray": move_to_left_tray,
    "move_to_right_tray": move_to_right_tray,
    "move_to_packaging": move_to_packaging,
    "move_to_idle": move_to_idle,
}


def prepare_experiment(home_gripper_client, grasp_client, move_group) -> None:
    home_gripper(home_gripper_client)
    move_to_home(move_group)
    move_to_idle(move_group)
    grasp(grasp_client, 0.035)
    print("Press ENTER to grasp bowl ...")
    input()
    grasp(grasp_client, 0.03)
    print("Press ENTER to continue ...")
    input()


def end_experiment(home_gripper_client, grasp_client, move_group) -> None:
    move_to_idle(move_group)
    print("Press ENTER to release bowl ...")
    input()
    grasp(grasp_client, 0.035)
    print("Press ENTER to when you removed the bowl ...")
    input()
    move_to_home(move_group)
    home_gripper(home_gripper_client)