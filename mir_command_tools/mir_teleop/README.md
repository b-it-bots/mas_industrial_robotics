Extracted temporarily from [mas_industrial_robotics](https://github.com/b-it-bots/mas_industrial_robotics/tree/melodic/mir_command_tools/mir_teleop) to port to ROS2. This package can be deleted once `mas_industrial_robotics` is ported to ROS2.

![youbot joypad description](docs/youbot_joypad_description.png)

### Compile
```
colcon build --symlink-install --packages-select mir_teleop
```

### Run
```
ros2 launch mir_teleop teleop_joypad.launch.py
```

### Test
Hold the `Deadman` switch (RB) and move the left axis for translation and right axis for rotation.
```
ros2 topic echo /joy
ros2 topic echo /cmd_vel
```
