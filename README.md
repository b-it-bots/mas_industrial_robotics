# mas_industrial_robotics - humble/bringup

The branch contains the necessary packages to bringup the robot with driver controllers.

## Setup

- Create a workspace
```bash
mkdir -p ~/mir/src
```

- Clone the humble/bringup branch.
```bash
cd ~/mir/src

git clone -b humble/devel https://github.com/vamsikalagaturu/mas_industrial_robotics.git
```

- Clone the dependency packages
```
cd ~/mir/src

vcs import < mas_industrial_robots/mir.repos
```
