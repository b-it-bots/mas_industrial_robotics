mir_refbox_parser
---------------

# Description
This node contains components to store the world model of the environment and the goal of the tasks in the knowledge base of the robot.
World model is defined as the locations and properties of different moveable
objects(eg: robot, box etc) in the environment.
The world model node has to subscribe to the incoming data stream which informs
of the current state of the world.(Eg : Refree box)
It then has to store this value in the any format which can be utilized for
other components.


## Required components for running
`sudo apt-get install flex ros-indigo-mongodb-store ros-indigo-tf2-bullet freeglut3-dev`

Rosplan :
https://github.com/KCL-Planning/ROSPlan

The following folders are not required they can be avoided by `$> touch CATKIN_IGNORE`
inside the respective directories
- rosplan_demos
- rosplan_interface_mapping
- rosplan_interface_movebase



**Input(s):**
  * `event_in`: trigger to start the node.
  * `refbox`: the ros topic string format on which the BTT, BNT, BPT, BMT, CBT messages is being published

**Output(s):**
  * `e_status`: Returns if the data storage ws successful

**Relevant parameter(s):**

**Diagram:**
![mir_refbox_parser](/ros/doc/node.png)

### Usage
1. Launch the component (example):
  ```roslaunch mir_refbox_parser refbox_parser.launch```

### Testing
1. We need the Knowledge base service running to test the loading of knowledge
   base
```roslaunch mir_pddl_problem_generator
rosplan_knowledge_base_example.launch```
currently this node is available in the
[unmerged_packages_for_testing](https://mas.b-it-center.de/gitgate/dnair2s/unmerged_packages_for_testing/tree/master/task_planning/mir_pddl_problem_generator)

Domain used is
[here](https://mas.b-it-center.de/gitgate/dnair2s/unmerged_packages_for_testing/blob/master/task_planning/mir_knowledge/common/pddl/general_domain/domain.pddl)

Before running the test the file has to be copied to the
mir_pddl_problem_generator/ros/test/example_domain folder.

2. For testing navigation we need the navigation action server
```roslaunch mir_move_base_safe move_base.launch```
