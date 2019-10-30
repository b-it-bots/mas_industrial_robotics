# Description
This package moves the base of a robot to a target pose.

## 'move_base' node:
This component moves the base of a robot to a target pose and provides
feedback of its result, i.e. 'e_success' if the pose was reached or 'e_failure'
otherwise. The component also preempts the motion towards the target
pose if an 'e_stop' message is received.

**Assumptions:**
  * The target pose is reachable.

**Input(s):**
  * `pose_in`: The target pose to which the robot's base should be moved.

**Output(s):**
  * `goal`: The navigation goal for the robot's base.

** Diagram **

![Move Base][move_base]

### Usage
1. Launch the component (example):

  ``` roslaunch mcr_move_base move_base_example.launch```

  [**Note**: You will probably need to create your own launch file and configure it according to your needs.]
1. Subscribe to the result(s) of the component:

  ```rostopic echo /move_base_wrapper/event_out```
1. Publish the target pose (example):

  ```
  rostopic pub /move_base_wrapper/pose_in geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 0.2, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} }}'
  ```
1. Toggle the component:
  1. To start the component:

    ```rostopic pub /move_base_wrapper/event_in std_msgs/String 'e_start'```
  1. To stop the component:

    ```rostopic pub /move_base_wrapper/event_in std_msgs/String 'e_stop'```

[move_base]: https://mas.b-it-center.de/gitgate/mas-group/mas_common_robotics/tree/hydro/mcr_navigation/mcr_move_base/ros/doc/move_base.png "Move Base"
