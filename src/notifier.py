import requests
from state_machine import ArmProgram, GazeProgram

STATE_MACHINE_URL   = "http://state_machine.local/arm_location"

def notify_arm_location(arm_location: str):
    try:
        requests.post(STATE_MACHINE_URL, json={"arm_location": arm_location}, timeout=0.5)
    except Exception as e:
        print("ERROR while sending data to state machine: ", str(e))
        pass  # log or retry as you wish
