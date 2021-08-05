#!/bin/bash

SESSION="bringup"

#allow re-launch
/usr/bin/tmux has-session -t $SESSION 2> /dev/null && /usr/bin/tmux kill-session -t $SESSION
/usr/bin/tmux -2 new-session -d -s $SESSION

echo "Exporting and sourcing necessary stuff"

/usr/bin/tmux send-keys -t $SESSION.0 "export HOME=/home/robocup" C-m
/usr/bin/tmux send-keys -t $SESSION.0 "cd" C-m
/usr/bin/tmux send-keys -t $SESSION.0 "source ~/.bashrc" C-m

# Use either block no. 1 or 2 depending on the use case. 2 launches gripper
# separately. This generally has higher chance of gripper coming up faster
# compared to 1 which is desirable during competitions.


# 1) unified bringup
echo "Launch bringup"
/usr/bin/tmux send-keys -t $SESSION.2 "bringup" C-m


# 2) gripper launched separately from bringup
# echo "Launching gripper"
# /usr/bin/tmux send-keys -t $SESSION.0 "roslaunch mir_gripper_controller dynamixel_gripper.launch" C-m
# echo "Launch bringup"
# /usr/bin/tmux split-window -h
# sleep 10s
# /usr/bin/tmux send-keys -t $SESSION.1 "export HOME=/home/robocup" C-m
# /usr/bin/tmux send-keys -t $SESSION.1 "cd" C-m
# /usr/bin/tmux send-keys -t $SESSION.1 "source ~/.bashrc" C-m
# /usr/bin/tmux send-keys -t $SESSION.1 "bringup" C-m
