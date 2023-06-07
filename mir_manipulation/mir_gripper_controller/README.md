# Gripper controller
Package containing code to control two finger gripper with Dynamixel 12A motors and OpenRB-150 controller.

More details on the controller and corresponding code can be found [here](https://github.com/b-it-bots/youbot_dynamixel_gripper_controller).

> Note: Please make sure to follow all the steps and setup the Dynamixel correctly as described in the above link.

## Requirements
* python3
* ROS Noetic
* pySerial

## Getting started
1. Make sure to uplaod the latest [code](https://github.com/b-it-bots/youbot_dynamixel_gripper_controller/tree/master/openrb_150_controller) on to the controller.
5. Run `roscore` command.
5. Run `dynamixel_gripper_controller_ros` in the `ros/scripts` directory.
6. To control the gripper publish to the `/arm_1/gripper_command` topic, e.g. `rostopic pub /arm_1/gripper_command std_msgs/String "command: 0.0"`
7. To see the state of the gripper subscribe to the `/arm_1/gripper_feedback`, e.g. `rostopic echo /arm_1/gripper_feedback`.

## Usage
* One can publish `0.0 (open)` or `1.0 (close)` command to the gripper.
* For more specific position control, one can publish a value between `0.0 and 1.0`.
* To further open the gripper, one can publish a value between `-1.5 and 0.0`.
* One can subscribe the state of the gripper, the format consists of the following fields: `state` (`GRIPPER_CLOSE`, `OBJECT_GRASPED`, `OBJECT_SLIPPED`, `GRIPPER_OPEN`, `GRIPPER_INTER`), `parsing_error` (0 - false, 1 - true), `last_command` (0, 1).
