## mir_perceive_aruco_cube

Detects aruco cube and publishes its pose.
This package is meant to be used as mock perceive instead of real perceive. This
might help in integrating the rest of the pipeline (pick, place, etc) when actual
perception is not yet ready.

Input: raw image and point cloud
Output: geometry_msgs/PoseStamped

### Events:
`e_trigger`: Tries to find an aruco cube in current view and publishes its pose
along with an output event (`e_success` or `e_failure`).

