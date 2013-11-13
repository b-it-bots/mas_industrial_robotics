This program is designed to detect the largest blob (object) in the scene and move the robot and arm in place to properly and safely interact with the object. 

Any questions or problems please contact the author Matthew Roscoe at: mat.roscoe@unb.ca

ROS dependancies: 
[arm_navigation_msgs] http://ros.org/wiki/arm_navigation
[brics_actuator] http://www.ros.org/wiki/cob_common

## Building: 

`$ rosmake mir_visual_servoing`

## Running: 

`$ roslaunch mir_visual_servoing visual_servoing.launch`

NOTE: Requires "usb_cam" to be running in order to work.

## State Machine Output

The following is the output in `byte` that tells the calling state machine what the status of the visual servoing application is.

`SUCCESS = 0`
`FAILED = -1`
`TIMEOUT = -2`
`LOST_OBJ = -3`
