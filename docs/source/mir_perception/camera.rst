.. _camera:

Camera
=======

Tower-mount camera calibration
-----------------------

On workstation or your PC

1. To shh the youbot (in all terminals):

  .. code-block:: bash

      yb4

  .. note::

      alias yb4=ssh -X robocup@youbot-brsu-4-pc2

2. Export the youbot ssh alias

  .. code-block:: bash

      export_yb4

  .. note::

      alias export_yb4=export ROS_MASTER_URI=http://youbot-brsu-4-pc2:11311

On robot

3. Launch the robot

  .. code-block:: bash

      roslaunch mir_bringup robot.launch

4. Run calibration

  .. code-block:: bash

      roslaunch mir_calibrate_pick calibrate_pick.launch

  A small gui window to adjust the pose of the end-effector in terms of XYZRPY will appear.

5. Place a small round object on the ground

On workstation or your PC

6. Run rviz

  .. code-block:: bash

    rosrun rviz rviz

7. Load the youbot.rviz config and set the global frame to `base_link`

    In perception add `raw_color_pcl`.  You can see an arrow pointing away from the robot.

8. Adjust the values in the pose mockup gui, so that the beginning of the arrow matches the center of the round object.

  .. note::

      Make sure the object is not too far away from the robot and the Z value is slightly above the object.


  .. figure:: images/camera_calib_side.png   
    :align: center

    Camera calibration example side view


  .. figure:: images/camera_calib_top.png   
    :align: center

    Camera calibration example top view

On robot

9. Test the calibration in another terminal

  .. code-block:: bash

      rosrun mir_calibrate_pick calibrate_pick_client_test.py

    The robot will move towards the gripper and move the end-effector of the arm close the object based on the given offset.

10. If the final end-effector position is not properly aligned to the desired goal position, in the robot navigate to the robot urdf configuration and edit the `robot.urdf.xacro`

  .. code-block:: bash

      /ros/noetic/robocup/src/mas_industrial_robotics/mir_robots/mir_hardware_config/youbot-brsu-2/urdf/robot.urdf.xacro

  .. code-block:: bash

      <xacro:realsense_d435 name="arm_cam3d" parent="base_link">
        <origin xyz="0.30 -0.05 0.80" rpy="0.00 1.137 0.0" />
      </xacro:realsense_d435>

  Adjust the values of xyz and rpy to account for the offset according to the values set in step 8 and repeat from step 8.

11. If the final end-effector position is properly aligned to the desired goal position, the camera calibration is complete.

12. Terminate all operations and relaunch the robot to continue.


.. _realsense2_camera:

RealSense2 camera
------------------

How to use the RealSense2 camera

1. Installation

  Go to the intel-ros github page. Clone the realsense repository in your catkin workspace inside src:

  .. code-block:: bash

    git clone git@github.com:intel-ros/realsense.git

2. Camera Output

  Run the following to get access to the camera:

  .. code-block:: bash

    roslaunch realsense2_camera rs_rgbd.launch

  Open rviz to visualize the camera output.

3. Configure camera output (OPTIONAL)

  Run the following to open the rviz configuration window:

  .. code-block:: bash

    rosrun rqt_reconfigure rqt_reconfigure

  You can also try to change the "octree_resolution" value:

  .. code-block:: bash

    cd *catkin workspace*/src/mas_perception/mcr_scene_segmentation/ros/config

4. Setup Base Frame

  Run the following:

  .. code-block:: bash

    rosrun tf static_transform_publisher x y z roll pitch yaw base_link camera_link 100

  where x, y, z are the distances and roll, pitch, yaw are the rotations from the base_link to the camera_link.

  To visualize your frames in rzviz, add the TF feature in the rviz menu.

5. Save Point Clouds

  If it's your first time saving point clouds, you need to choose where you want to save them and enable data collection:

  .. code-block:: bash

    cd *catkin workspace*/src/mas_perception/mcr_scene_segmentation/ros/launch

  Change the value of "dataset_collection" from "false" to "true". Change value of "logdir" from "/temp/
  to the path in your computer where you want to save the files.

  Run the following to get access to the point clouds given by the camera:

  .. code-block:: bash

    roslaunch mcr_scene_segmentation scene_segmentation.launch input_pointcloud_topic:=/camera/depth_registered/points

  Publish the message 'e-start':

  .. code-block:: bash

    rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String "data: 'e_start'"

  Publish the message 'e-add-cloud-start':

  .. code-block:: bash

    rostopic pub /mcr_perception/scene_segmentation/event_in std_msgs/String "data: 'e_add_cloud_start'"

  This last one will save the current point cloud of the observed object in your system.

  .. warning::

    Sometimes the camera won't save the point cloud (don't worry, not your fault).
    Just try a different position for the object until it works.

6. Visualize Point Cloud

  Run the following in the folder where you saved the point clouds:

  .. code-block:: bash

    pcl_viewer *.pcd file you want to open*