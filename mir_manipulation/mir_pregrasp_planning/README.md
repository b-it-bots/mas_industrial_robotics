# Description
This package contains components to compute a *pre-grasp* pose, based on a target pose,
for the end effector of a robot's manipulator.

## 'simple_pregrasp_planner' node:
This component computes a modified pose based on the constraints imposed by
the youBot's manipulator and gripper. Namely, the manipulator cannot reach
arbitrary 6D poses due to having only 5 degrees of freedom; furthermore, the
gripper size doesn't allow for side grasps when the object to be grasped has
a small height (defined as a height lower than the 'height_threshold').

The pose is modified further if the object is not standing up (or if it is
standing but has a 'small' height). This last modification is, for a single
rotation axis (reference_axis), by adding an offset and limiting the rotation
to be within certain range. If no rotation_range is specified, the pose is not
modified any further (except for the specified offset).

**Assumptions:**
  * The object's pose has the X axis pointing up, when the object is standing on
  a surface.

**Input(s):**
  * `pose_in`: The target pose from which to base the calculation of the *pre-grasp*
  pose.

**Output(s):**
  * `pose_out`: The *pre-grasp* pose.
  * `sampling_parameters`: A message specifying the parameters, and constraints,
  of the pose to be sampled around an object, if any.
  * `grasp_type`: The type of grasp selected given a particular `pose_in`,
  e.g. top grasp, side grasp.

**Parameter(s):**
  * `height_threshold`: Tolerance to decide whether an object should be re-oriented,
  based on its height (in meters).
  * `reference_axis`: Rotation axis of the pose to be modified (e.g. x, y, z).
  * `rotation_offset`: Rotation offset to add to the reference_axis of the pose
  (in degrees).
  * `rotation_range`: Range of rotation allowed as a two-element list, e.g.:
  [rotation_range[0] - rotation_range[1]] (in degrees).
  The rotation range might be specified with its first value (minimum)
  greater than the second value (maximum) to cover a range within the
  circle that passes through zero (e.g.: The rotation range of [270- 90] covers
  from 270 to 359 and then from 0 to 90).
  * `min_distance_to_object`: Closest distance the gripper should be to the object
  (in meters).
  * `max_distance_to_object`: Farthest distance the gripper should be to the object
  (in meters).
  * `loop_rate`: Node cycle rate (in hz).

** Diagram **

![Simple grasp planner][simple_grasp_planner]


### Usage
1. Launch the component (example):

  ``` roslaunch mir_pregrasp_planning simple_pregrasp_planner_example.launch```

  [**Note**: You will probably need to create your own launch file and configure it according to your needs.]
1. Subscribe to the result(s) of the component:

  ```rostopic echo /simple_pregrasp_planner/pose_out```

  ```rostopic echo /simple_pregrasp_planner/grasp_type```

  ```rostopic echo /simple_pregrasp_planner/sampling_parameters```
1. Publish the target pose (example):

  ```
  rostopic pub /simple_pregrasp_planner/pose_in geometry_msgs/PoseStamped '{header: {frame_id: "base_link"}, pose: {position: {x: 3.5, y: 2.1, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} }}'
  ```
1. Toggle the component:
  1. To start the component:

    ```rostopic pub /simple_pregrasp_planner/event_in std_msgs/String 'e_start'```
  1. To stop the component:

    ```rostopic pub /simple_pregrasp_planner/event_in std_msgs/String 'e_stop'```

### Usage (with GUI)
1. Launch the component (example):

  ``` roslaunch mir_pregrasp_planning simple_pregrasp_planner_example.launch```

  [**Note**: You will probably need to create your own launch file and configure it according to your needs.]
1. Subscribe to the result(s) of the component:

  ```rostopic echo /simple_pregrasp_planner/pose_out```

  ```rostopic echo /simple_pregrasp_planner/grasp_type```

  ```rostopic echo /simple_pregrasp_planner/sampling_parameters```
1. Launch the GUI:

  ```roslaunch mir_pregrasp_planning pose_mock_up_gui.launch```
1. Toggle the component:
  1. To start the component:

    ```rostopic pub /simple_pregrasp_planner/event_in std_msgs/String 'e_start'```
  1. To stop the component:

    ```rostopic pub /simple_pregrasp_planner/event_in std_msgs/String 'e_stop'```

## 'pregrasp_planner_pipeline' node:
This component takes an object's pose (target pose) and depending on different
constraints (e.g. the object is standing or laying horizontally, object's height
is short, etc.), it computes a set of poses, within a threshold, to reach the
target pose with a manipulator's end effector. From this set of poses, this component
selects a valid pose (i.e. one that reaches the target pose) and it computes the
joint configuration to reach that valid pose.

It uses the following nodes:
  * (mir_pregrasp_planning) `simple_pregrasp_planner`.
  * (mcr_pose_generation) `pose_generator`.
  * (mcr_pose_selection) `reachability_pose_selector`.

The component serves as a configurator/coordinator, i.e. it sets the required
parameters for all the components and starts/stops them accordingly.

**Assumptions:**
  * The target pose is within the reach of the robot's manipulator.

**Input(s):**
  * `target_pose`: The pose where the manipulator's end effector is required to be.

**Output(s):**
  * `selected_pose`: The pose of the manipulator's last joint so that the end effector
  reaches the `target_pose`.
  * `configuration_out`: The joint position values for the manipulator to reach
  the `selected_pose`.

**Relevant parameter(s):**
  * `simple_grasp_planner`
    * `height_threshold`: Tolerance to decide whether an object should be re-oriented,
    based on its height (in meters).
    * `rotation_offset`: Rotation offset to add to the reference_axis of the pose
    (in degrees).
    * `rotation_range`: Range of rotation allowed as a two-element list, e.g.:
    [rotation_range[0] - rotation_range[1]] (in degrees).
  * `pose_generator`
    * `gripper_matrix`: A 4x4 matrix that defines the pose of the end effector relative
    to the pose of the manipulator's last joint, i.e. this should be modified for
    different grippers.

** Diagram **
![Pregrasp planner pipeline][pregrasp_planner_pipeline]

### Usage (with GUI)
**Note:** This component requires a robot model to be uploaded and MoveIt! to be running.

1. Launch the component (example):

  ```roslaunch mir_pregrasp_planning pregrasp_planner_pipeline_example.launch```

  [**Note**: You will probably need to create your own launch file and configure it according to your needs.]
1. Subscribe to the result(s) of the component:

  ```rostopic echo /pregrasp_planner_pipeline/configuration_out```

  ```rostopic echo /pregrasp_planner_pipeline/selected_pose```

  ```rostopic echo /pregrasp_planner_pipeline/grasp_type```
1. Toggle the component:
  1. To start the component:

    ```rostopic pub /pregrasp_planner_pipeline/event_in std_msgs/String 'e_start'```
  1. To stop the component:

    ```rostopic pub /pregrasp_planner_pipeline/event_in std_msgs/String 'e_stop'```

[simple_grasp_planner]: https://mas.b-it-center.de/gitgate/mas-group/mas_industrial_robotics/tree/hydro/mir_manipulation/mir_pregrasp_planner/ros/doc/simple_grasp_planner.pdf "Simple grasp planner"
[pregrasp_planner_pipeline]: https://mas.b-it-center.de/gitgate/mas-group/mas_industrial_robotics/tree/hydro/mir_manipulation/mir_pregrasp_planner/ros/doc/pregrasp_planner_pipeline.pdf "Pregrasp planner pipeline"
