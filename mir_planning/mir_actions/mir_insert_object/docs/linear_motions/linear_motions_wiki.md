# Description
This package contains launch files to execute different types
of arm motions (e.g. planned, Cartesian).

## 'linear_motion_with_interpolatino' node:
This component moves the arm in a linear motion using the
'mcr_trajectory_generation' package, specifically using the linear interpolation
implementation.

**Input(s):**
  * `start_pose`: The pose where the manipulator's end effector will start the
  trajectory.
  * `goal_pose`: The pose where the manipulator's end effector will end the trajectory.

**Output(s):**
  * The arm motion.

**Relevant parameter(s):**
  * `linear_offset`: A linear offset in X, Y, Z (in meters) to shift the end-effector
  to another pose.

** Diagram **
![Linear motions with interpolation][linear_motions_with_linear_interpolator]

### Usage
1. Launch the components (example):

  ```roslaunch mir_arm_motions poses_to_move_down.launch```

  ```roslaunch mir_arm_motions poses_to_move_up.launch```

  ```roslaunch mir_arm_motions linear_motions_with_interpolation_example.launch```

  [**Note**: You will probably need to create your own launch file and configure it
  according to your needs.]
1. Start the component to generate the poses (to go up):

  ```rostopic pub /poses_to_move_up/event_in std_msgs/String "e_start"```

  or (to go down)

  ```rostopic pub /poses_to_move_down/event_in std_msgs/String "e_start"```

1. Subscribe to the result(s) of the component:

  ```rostopic echo /linear_interpolator_demo/event_out```

1. Toggle the component:
  1. To start the component:

      ```rostopic pub /linear_interpolator_demo/event_in std_msgs/String "e_start"```
  1. To stop the component:

      ```rostopic pub /dmp_trajectory_demo/event_in std_msgs/String "e_stop"```
1. If the result of the linear_interpolator_demo was a `e_success`, then the arm can be moved with
the following command:

  ```rostopic pub /linear_interpolator_demo_trajectory_executor/event_in std_msgs/String "e_start"```


## 'linear_motions_with_cartesian_controller' node:
This component moves the arm in a linear motion using the
'mcr_guarded_approach_pose' package, specifically using the Cartesian controller
implementation.

**Input(s):**
  * `start_pose`: The pose where the manipulator's end effector will start the
  trajectory.
  * `goal_pose`: The pose where the manipulator's end effector will end the trajectory.

**Output(s):**
  * The arm motion.

**Relevant parameter(s):**
  * `linear_offset`: A linear offset in X, Y, Z (in meters) to shift the end-effector
  to another pose.

** Diagram **
![Linear motions with Cartesian controller][linear_motions_with_cartesian_controller]

### Usage
1. Launch the components (example):

  ```roslaunch mir_arm_motions poses_to_move_down.launch```

  ```roslaunch mir_arm_motions poses_to_move_up.launch```

  ```roslaunch mir_arm_motions linear_motions_with_cartesian_controller_example.launch```

  [**Note**: You will probably need to create your own launch file and configure it
  according to your needs.]
1. Start the component to generate the poses (to go up):

  ```rostopic pub /poses_to_move_up/event_in std_msgs/String "e_start"```

  or (to go down)

  ```rostopic pub /poses_to_move_down/event_in std_msgs/String "e_start"```

1. Subscribe to the result(s) of the component:

  ```rostopic echo /cartesian_controller_demo/event_out```

1. Toggle the component:
  1. To start the component:

      ```rostopic pub /cartesian_controller_demo/event_in std_msgs/String "e_start"```
  1. To stop the component:

      ```rostopic pub /cartesian_controller_demo/event_in std_msgs/String "e_stop"```

[linear_motions_with_linear_interpolator]: https://mas.b-it-center.de/gitgate/mas-group/mas_industrial_robotics/tree/hydro/mir_manipulation/mir_arm_motions/ros/doc/linear_motions_with_linear_interpolator.png "Linear motions with interpolation"
[linear_motions_with_cartesian_controller]: https://mas.b-it-center.de/gitgate/mas-group/mas_industrial_robotics/tree/hydro/mir_manipulation/mir_arm_motions/ros/doc/linear_motions_with_cartesian_controller.png "Linear motions with Cartesian controller"
