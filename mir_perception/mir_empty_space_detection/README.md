# mir_empty_space_detection

Detects empty space on a workstation using point cloud library.

## Usage

- Launch the perceiver with
  ```
  roslaunch mir_empty_space_detection empty_space_detector.launch
  ```

- See the response with
  ```
  rostopic echo /mir_perception/empty_space_detector/event_out 
  ```

- Trigger the detection with
  ```
  rostopic pub /mir_perception/empty_space_detector/event_in std_msgs/String "data: 'e_start'" -1
  ```

## Topics

### In
- `/mir_perception/empty_space_detector/event_in`
- `/mir_perception/empty_space_detector/input_point_cloud` -> `/arm_cam3d/depth_registered/points`

### Out
- `/mir_perception/empty_space_detector/output_pose`
- `/mir_perception/empty_space_detector/event_out`
- `/mir_perception/empty_space_detector/output_point_cloud`
