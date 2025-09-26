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
6. To control the gripper publish to the `/arm_1/gripper_command` topic, e.g.:
    ```bash
    rostopic pub /arm_1/gripper_command mir_manipulation_msgs/GripperCommand "command: 0.0"
    ```
7. To see the state of the gripper subscribe to the `/arm_1/gripper_feedback`, e.g. `rostopic echo /arm_1/gripper_feedback`.

## Usage
* One can publish `0.0 (open)` or `1.0 (close)` command to the gripper.
* For more specific position control, one can publish a value between `0.0 and 1.0`.
* To further open the gripper, one can publish a value between `-1.5 and 0.0`.
* One can subscribe the state of the gripper, the format consists of the following fields: `state` (`GRIPPER_CLOSE`, `OBJECT_GRASPED`, `OBJECT_SLIPPED`, `GRIPPER_OPEN`, `GRIPPER_INTER`), `parsing_error` (0 - false, 1 - true), `last_command` (0, 1).

## Troubleshooting

### Board at /dev/youbout/gripper not found
* Check if the board is connected to the computer.
* Check if the board is connected to the correct USB port.
* Check the `idProduct` of the Open-RB board is properly set in /etc/udev/rules.d/80-youboot-brsu-2-devices.rules
* To get the `idProduct` of the Open-RB board:
    - run `dmesg | grep OpenRB`, you will see something like this:
        ```bash
            [    6.449181] usb 1-2.3: Product: OpenRB-150
        ```
    - run `dmesg | grep "usb 1-2.3"`: in the generated output, find the line that looks like:
        ```bash
             usb 1-2.3: New USB device found, idVendor=2f5d, idProduct=2202, bcdDevice= 1.00
        ```
    - From this, get the `idProduct` and update it in the file `/etc/udev/rules.d/80-youboot-brsu-2-devices.rules`
    - Then run `sudo udevadm control --reload`
    - Now, unplug and plug the board again. It should be detected now.
    - To test, run the `serial_interface.py` file in the `ros/src` directory. It should close and open the gripper.

