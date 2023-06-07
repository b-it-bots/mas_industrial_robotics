# gripper_control
Repository containing code for Teensy microcontroller and ROS interface for transferring gripper commands and feedback data. The software is designed in a way to prevent occurence of the overcurrent in the Dynamixels and enable slippage detection.

## Requirements
* python3
* ROS Noetic
* pySerial

## Getting started
1. Download [Arduino IDE](https://www.arduino.cc/en/software) (do not use Ubuntu software manager!).
2. Download Teensy plugin for Arduino IDE [here](https://www.pjrc.com/teensy/teensyduino.html).
3. Download libraries for Arduino IDE: Dynamixel2Arduino, CircularBuffer, Arduino_JSON, this is done in the Arduino IDE (Tools->Manage Libraries).
4. Open file `control_with_comm.ino` in the `teensy/src` directory, compile it and upload on the microcontroller.
5. Run `roscore` command.
5. Run `gripper_controller.py` in the `ros/src` directory.
6. To control the gripper publish to the `/arm_1/gripper_command` topic, e.g. `rostopic pub /arm_1/gripper_command std_msgs/String "data: '0'"`
7. To see the state of the gripper subscribe to the `/arm_1/gripper_feedback`, e.g. `rostopic echo /arm_1/gripper_feedback`.

## Setting up the Dynamixel
- Please follow the guide [here](https://emanual.robotis.com/docs/en/parts/interface/usb2dynamixel/).
- For setting the baudrate install [here](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/).

## Usage
* One can publish 0 (open) or 1 (close) command to the gripper.
* One can subscribe the state of the gripper, the format consists of the following fields: `state` (`GRIPPER_CLOSE`, `OBJECT_GRASPED`, `OBJECT_SLIPPED`, `GRIPPER_OPEN`), `parsing_error` (0 - false, 1 - true), `last_command` (0, 1).

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