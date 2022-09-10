# MIR Object Recognition ROS2 Code Base

## Features impelemented

* Multimodal object recognition node as a Lifecycle node.
* A universal lifecycle controller node.
* Composition of nodes.
* Dynamic paramter reconfiguration using rqt_reconfigure for runtime manipulation of parameters.
* Integrated RGB Object recogntion using YOLOv5 from [b-it-bots]().

> Note: The code is only tested with the RGB recognition. The depth recognition is not tested due to the lack of 3D object recognition models.


## Environmental setup

> Note: Ubuntu 20.04 LTS(Focal Fossa) is recommended for this codebase to work.

### Install ROS2 Rolling
```
sudo apt install ros-rolling-desktop
```
For more details, see [ROS2 Rolling Installation](https://docs.ros.org/en/rolling/Installation/Alternatives/Ubuntu-Development-Setup.html).

### Source the setup script
```
source /opt/ros/rolling/setup.bash
```

### Install dependencies
* cv-bridge
```
sudo apt-get install ros-rolling-cv-bridge
```

* yaml-cpp-vendor
```
sudo apt install ros-rolling-yaml-cpp-vendor*
```

* rqt-reconfigure
```
sudo apt install ros-rolling-rqt-reconfigure
```

### Clone the necessary packages into the workspace

```
mkdir ~/mir_object_recognition/src
cd ~/mir_object_recognition/src

git clone --branch rolling-devel https://github.com/HBRS-SDP/ss22-ros2-perception.git .

git clone --branch foxy-devel https://github.com/HBRS-SDP/mas_perception_msgs.git
```

> Note: If you want to use the bag file for RGB image and Pointcloud data, skip the next step.

### Setup the RealSense SDK and ROS2 wrapper for RealSense cameras

```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev

git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-beta
```

### Build the workspace
* Packages that will be built are:
    * `mas_perception_msgs`
    * `realsense2_camera_msgs`
    * `lifecycle_controller`
    * `mir_rgb_object_recognition_models`
    * `mir_object_recognition`
    * `mir_recognizer_scripts`
    * `realsense2_camera`
    * `realsense2_description`

```
colcon build 
source install/setup.bash
```

* Refer to our [wiki](https://github.com/HBRS-SDP/ss22-ros2-perception/wiki/Issues) to troubleshoot any known issues with the build.


## Steps to run

> Note: Make sure to source the ROS rolling and devel in all the terminals

**Step 0:**
* Navigate to the worskpace directory and source the ROS and workspace in all the terminals.
```
cd ~/mir_object_recognition
source /opt/ros/rolling/setup.bash
source install/setup.bash
```

**Step 1:**

* If using a realsense camera, connect the camera to your system where you are running this codebase and run the following command in terminal 1:
```
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true pointcloud.ordered_pc:=true depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30
```
* The `ros2-beta` branch of `realsense2_camera` package has a bug that doesn't set the `pointcloud.ordered_pc` parameter to true. So, we have to set it manually using the `ros2 param` command.
```
ros2 param set /camera pointcloud.ordered_pc true
```
* To align the pointcloud depth properly, set the below parameter to true.
```
ros2 param set /camera align_depth.enable true
```

Or

* If running in the simulation, run the bag file in loop in terminal 1 using the command below:
```
ros2 bag play -l bag_files/bag_file_name
```
* If you dont have the bag file, download one from the below link and save it in `~/mir_object_recognition/bag_files`:
```
https://drive.google.com/file/d/1okPBwca5MgtF6kc3yL3oOEA8TWGFyu-0/view
```

**Note:**
* In order for the object recognition to work properly, the RGB image and the Pointcloud data should be of same size and in sync.

* If you want to collect a bag file and use if for later, use the following command in terminal to record the bag file with the required topics:
```
ros2 bag record /camera/color/camera_info /camera/color/image_raw /camera/depth/color/points /clock /tf /tf_static
```

**Step 2:**
* If you are using a realsense camera, you have to publish the tf link between the camera and the base_link in another terminal:
```
ros2 run tf2_ros static_transform_publisher 0.298 -0.039 0.795 0.0 1.16 -0.055  base_link camera_link
```
* Change the tf values according to your camera setup.

**Step 3:**

* Run the multimodal_object_recognition launch file in terminal 2 using the command below (make sure to source your workspace).
```
ros2 launch mir_object_recognition multimodal_object_recognition.launch.py 
```
* Once the node is launch is up and running, move to next step.

**Step 4:**
* Run the rqt_reconfigure to dynamically change parameters via gui in terminal 3 using the command below:
```
ros2 run rqt_reconfigure rqt_reconfigure
```
* Click 'Enter' after chaning any input field values.


**Step 5:**

* Run the lifecycle_controller in terminal 4 using the command below.

```
ros2 run lifecycle_controller lifecycle_controller --ros-args -p lc_name:=mmor
```
* lifecycle_controller needs the lifecycle node name as a parameter to run. 
* Here, we are passing `mmor` for our multimodal_object_recognition (mmor) node.

* To know more about how to use the lifecycle controller, refer to the [wiki]().

**Step 6:**

* Run the RGB recognizer script in terminal 5 using the command below:
```
ros2 launch mir_recognizer_scripts rgb_recognizer.launch.py
```

**Step 7:**
* Run the rviz2 to view the object recognition output and other relevant data in terminal 6 using the command below:
```
rviz2
```
* Once the rviz is open, load the `/home/vamsi/mir_object_recognition/src/mir_object_recognition/ros/rviz/mir_object_recognition.rviz` file to view the recognized objects and their poses.

**Step 8:**

To perform RGB object recognition, follow the steps below:
   
* The mmor node will be in unconfigured state by default.
* Change the state to Inactive by entering `C` in the `lifecycle_controller` terminal, during which all the parameters, publishers, subscribers and other configurations take place.
* Refresh the `rqt_reconfigure` gui to see the updated parameters.
* To start processing the data, change the `mmor` node state to Active by entering `A` in the `lifecycle_controller` terminal.
* The `mmor` node then process the image and point cloud data and publishes the recognized objects list, along with their poses and bounding boxes.
* The object recognition from RGB recognizer, bounding boxes and poses from pointcloud can be visualized in `rviz2`.
* To terminate the `mmor` node, enter `X` in the `lifecycle_controller` terminal, which will shut down the node.
* To know more about the process flow of this project, refer to the [wiki]().


> More details about the concepts, issues and resources can be found on the wiki page.
