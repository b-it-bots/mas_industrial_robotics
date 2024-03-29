## mir\_barrier\_tape\_detection

Detects black and yellow barrier tape on the floor. The detected barrier tape points are accumulated and only cleared if explicitly told to do so.

Input: 3D (colour) pointcloud (in camera frame) and RGB image
Output: 3D pointcloud with points corresponding to the yellow sections of the barrier tape, in the desired output frame (assumed to be base link)

### Launching Barrier Tape Dectection  
    
    roslaunch mir_barrier_tape_detection barrier_tape_detection.launch 


    rostopic pub /mir_perception/barrier_tape_detection/event_in std_msgs/String "data: 'e_start'"

### Events:
`e_start`: start detection of barrier tape
`e_stop`: stop detection of barrier tape
`e_reset`: clears detected barrier tape points

