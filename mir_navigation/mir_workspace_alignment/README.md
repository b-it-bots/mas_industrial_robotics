# mir_workspace_alignment

This module uses [laser_line_extraction](https://github.com/kam3k/laser_line_extraction)'s output to 
determine the position of the workspace. It then publishes a `geometry_msgs/PoseStamped`
message to where the robot should ideally be after correctly aligning.

## Install

- clone [laser_line_extraction](https://github.com/kam3k/laser_line_extraction)
- build that package
- clone this package
- build this package

## Execution
```
roslaunch mir_workspace_alignment workspace_alignment.launch
rostopic pub /workspace_aligner/event_in std_msgs/String "data: 'e_trigger'"
```
