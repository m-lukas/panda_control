# Panda Control

## Requirements

- Franka Panda Emika with libfranka and franka_ros installed from source
- pip installed
- rospy + requests installed globally
- FCI activated
- Robot in `Ready` state
- `panda_moveit` controller started
- panda_control must be installed in same catkin workspace as panda_moveit. Example path: `~/ws_moveit/src/panda_control`

> Hint: You can end the program using STRG+C - do NOT do this during active movements.

## How to Run

1. Open new Terminal tab
2. Run `rosrun panda_control src/main.py`

## Programs

Programs can be added as Python functions in `programs.py` and need to be added to the `PROGRAMS` directory in the same file. Programs need to be added manually to `controller.html` if Web-Control is desired. Else programs can be executed via `curl`.
