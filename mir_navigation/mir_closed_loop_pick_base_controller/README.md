# mir_closed_loop_pick_base_controller

Moves the base with `Twist` messages to move in front of the object.

Assumptions: The camera is between the gripper and the arm is looking at the
objects on a workstation. The objects are standing and they are supposed to pick
with side grasp.

## Setup
```
roslaunch mir_closed_loop_pick_base_controller clp_base_controller
```
