## Mir Empty space prediction

Launch the mir_empty_space_prediction from a new terminal
```
roslaunch mir_empty_space_prediction empty_space_predictor.launch
```

In order to set the ROI for workstations, open config-> params.yaml file and adjust the x_min, x_max, y_min and y_max of DEFAULT and trigger the empty space bounding box with the defined ROI by 

```
 rostopic pub -1 /empty_space_detector/event_in std_msgs/String e_empty
```

chang the values and relanch the lauch file untill you are satisfied with ROI

Once you are satisifed stop the trigger, copy the values of DEFAULT and save them as per the name of the workstaion 

```
rostopic pub -1 /empty_space_detector/event_in std_msgs/String e_stop

```

and move to the next workstation repeat the process for all the workstations