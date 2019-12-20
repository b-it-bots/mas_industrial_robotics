approach to follow for pick action (outdated)
==================================

1. as precondition for pick action, perception needs to be triggered before hand

plus youbot needs to be in front of a platform

2. pipeline

object_selector -> pregrasp_planner -> plan/execute arm motion -> close gripper -> move arm to HOLD position

3. input - output:

input: object to pick (string), object list (from perception)
output: success (boolean) + will pick the object (maybe, sometimes it fails)

4. about object selector

use the object selector component from:

/home/oscar/ros_ws/robocup/src/mas_common_robotics/mcr_perception/mcr_perception_selectors/ros/launch

(roslaunch mcr_perception_selectors object_selector.launch)

this component listens to object list topic comming from perception and stores the information of it (it has memory lets say)

it then listens to the topic event in which takes arguments about which object to select (string)

when it receives event in it publishes the pose of the object (pose stamped)

3. about pregrasp planner

generates graspable configurations based on tolerance zenith, azimuth parameters (pose array) and object center

3. about plan_arm_motion

receives pose and moves the arm to that pose
