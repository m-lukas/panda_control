import requests
import threading

STATE_MACHINE_URL   = "http://state_machine.local/arm_location"

def notify_arm_location(arm_location: str):
    def _notify():
        try:
            #requests.post(STATE_MACHINE_URL, json={"arm_location": arm_location}, timeout=0.5)
            print("Sending to State Machine:", arm_location)
        except Exception as e:
            print("ERROR while sending data to state machine: ", str(e))
            pass  # log or retry as you wish
    threading.Thread(target=_notify, daemon=True).start   
