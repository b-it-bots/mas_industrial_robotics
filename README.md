# MIR Perception ROS2 Code Base


## Steps to run

> Note: Make sure to source the ROS rolling and devel in all the terminals

### **Camera feed**

**Step 1:**

* If running in the simulation, run the bag file in loop in terminal 1 using the command below:
```
ros2 bag play -l bag_files/test_bagfile_#
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
