# mir_world_generation

Procedurally generates at work arena for gazebo simulation.
Generates
- `.xacro` file containing world model for gazebo simulator
- `map.pgm` and `map.yaml` files for occupancy grid for ros navigation
- `navigation_goals.yaml` file for industrial robotics `move_base_safe` action

Change the parameters for generation in `common/config/config.yaml` file.

## Usage

Generate all necessary files using `grid_based_generator.py`
```
roscd mir_world_generation/common/mir_world_generation
python grid_based_generator.py
```

Visualise all the generated files using example `.launch` file
```
roslaunch mir_world_generation sim.launch
```

## Examples

![3x4](docs/map_plus_gazebo_3x4.png)
![4x4](docs/map_plus_gazebo_4x4.png)
