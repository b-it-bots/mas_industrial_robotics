# MIR Perception ROS2 Code Base

## Features impelemented

* Main node as a Lifecycle node.
* A universal lifecycle controller node.
* Dynamic paramter reconfiguration using rqt_reconfigure for runtime manipulation of parameters.
* Plane detection
* Visualization of plane detection in rviz2

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

### Remove cv-bridge package (to avoid conficts while building packages)
```
sudo apt-get remove ros-rolling-cv-bridge
```

### Build 'mas_perception_messages' package
```
mkdir ~/sdp_ws/src
cd ~/sdp_ws/src
git clone --branch foxy-devel https://github.com/HBRS-SDP/mas_perception_msgs.git
cd ~/sdp_ws
colcon build --packages-select mas_perception_msgs
```

### Build 'mir_object_recognition' package

```
cd ~/sdp_ws/src
git clone --branch rolling-devel https://github.com/HBRS-SDP/ss22-ros2-perception.git .
cd ~/sdp_ws
colcon build 
source install/local_setup.bash
```

### Install rqt-reconfigure for dynamic parameter reconfiguration
```
sudo apt install ros-rolling-rqt-reconfigure
```

## Steps to run

> Note: Make sure to source the ROS rolling and devel in all the terminals

### **Camera feed**

**Step 1:**

* If running in the simulation, run the bag file in loop in terminal 1 using the command below:
```
ros2 bag play -l bag_files/test_bagfile_#
```
* If you dont have the bag file, download one from the below link:
```
https://drive.google.com/file/d/1uXe8FBUS5M-M9RHVYGZQHTINmPeFljZW/view?usp=sharing
```

**Step 2:**

* Run the multimodal_object_recognition launch file in terminal 2 using the command below (make sure to source your workspace).
```
ros2 launch mir_object_recognition multimodal_object_recognition.launch.py 
```
* Once the node is launch is up and running, move to step 3

**Step 3:**

* Run the lifecycle_controller in terminal 3 using the command below.

```
ros2 run mir_object_recognition lifecycle_controller 
```
* lifecycle_controller is by default set to work with multimodal_object_recognition (mmor) node.
* If you want to configure it for a different node:
```
ros2 run mir_object_recognition lifecycle_controller --ros-args -p lifecycle_node_name:=node_name
```
* After running the lifecycle_controller we can see the following output as shown below.

<img src="https://github.com/HBRS-SDP/ss22-ros2-perception/blob/rolling-devel/images/lc_cntrl_out.png" >

**Step 4:**

* Using the keyboard inputs, we can control the lifecycle_controller by changing states. The current state is also displayed.

<img src="https://github.com/HBRS-SDP/ss22-ros2-perception/blob/rolling-devel/images/lc_cntrl_state_chng.png" >

* Follow the below steps to perform plane detection:    
    * The mmor node will be in unconfigured state by default.
    * Change the state to Inactive by entering `C`, during which all the parameters, publishers, subscribers and other configurations take place.
    * To process the data, change the state to Active by entering `A`.
    * The node then process the point cloud data and perform plane detection.
    * The detected plane is published to the topic `output/debug_cloud_plane`.
    * To terminate the mmor ndoe, enter `X` which will shut down the node.

* If a lifecycle node is not available the following error is displayed. Kill the node and re-run once the lifecycle node is available.

<img src="https://github.com/HBRS-SDP/ss22-ros2-perception/blob/rolling-devel/images/lc_cntrl_out_error.png" >


**Step 5:**

* Run the rqt_reconfigure to dynamically change parameters via gui in terminal 4 using the command below:
```
ros2 run rqt_reconfigure rqt_reconfigure
```
* Click 'Enter' after chaning any input field values.


Step 6:

* Run the rviz2 to view the pointclouds and other relevant data in terminal 5 using the command below:
```
rviz2
```
* Once the rviz is open, load the `plane_det.rviz` to view the input pointcloud and the detected plane (in red color).


> More details about the concepts, issues and resources can be found on the wiki page.
