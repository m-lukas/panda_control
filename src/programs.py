from typing import Callable, Dict, List

from controls import DEFAULT_ARM_SPEED, move_to_pose
from notifier import notify_arm_location


left_tray_index = 0
right_tray_index = 0

packaging_containers: List[tuple] = []
current_container_index = 0
number_of_items_in_container = 0


class MoveToHandoverProgram:
    def __init__(self, target_1: tuple, target_2: tuple, speed: float):
      self.target_1 = target_1
      self.target_2 = target_2
      self.speed = speed


left_tray_handovers: List[MoveToHandoverProgram] = []
right_tray_handovers: List[MoveToHandoverProgram] = []


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
    notify_arm_location("packaging")

    # move to specific container
    c = packaging_containers[current_container_index]
    move_to_pose(move_group, c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7])
    number_of_items_in_container += 1
    if number_of_items_in_container == 8:
        current_container_index += 1
        number_of_items_in_container = 0
        # account for errors
    # rotate gripper
    # rotate gripper back
    # (move to packaging common)
    notify_arm_location("handover_finished")
    pass


def move_to_idle(move_group, speed=DEFAULT_ARM_SPEED):
    # move waiting position
    notify_arm_location("idle")
    pass


PROGRAMS: Dict[str, Callable] = {
    "move_to_left_tray": move_to_left_tray,
    "move_to_right_tray": move_to_right_tray,
    "move_to_packaging": move_to_packaging,
    "move_to_idle": move_to_idle,
}