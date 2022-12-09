# mas_industrial_robotics - humble/bringup

The branch contains the necessary packages to bringup the robot with driver controllers.

## Dependent packages
- control_msgs
- urg_node
- joy_node
- moveit_msgs
- xacro
- twist-mux
- joint-state-publisher
- joint-state-publisher-gui
- realsense sdk and realsense2-ros

## Setup

- Create a workspace
```bash
mkdir -p ~/mir/src
```

- Clone the humble/bringup branch.
```bash
cd ~/mir/src

git clone -b humble https://github.com/mas_industrial_robotics.git
```

- Clone the dependency packages
```
cd ~/mir/src

vcs import < mas_industrial_robots/mir.repos
```

- Launch the bringup
```bash
ros2 launch mir_bringup robot.launch.py
```
