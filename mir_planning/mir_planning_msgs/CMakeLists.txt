cmake_minimum_required(VERSION 2.8.3)
project(mir_planning_msgs)

find_package(catkin REQUIRED
    genmsg
    std_msgs
    actionlib_msgs
    diagnostic_msgs
    rosplan_knowledge_msgs
    rosplan_dispatch_msgs
    message_generation
)

add_message_files(
    DIRECTORY
        ros/msg
    FILES
        ObjectsAtLocation.msg
)

add_action_files(
    DIRECTORY
        ros/action
    FILES
        ExecutePlan.action
        GenericExecute.action
        Plan.action
)

add_service_files(
    DIRECTORY
        ros/srv
    FILES
        ReAddGoals.srv
)

generate_messages(DEPENDENCIES
    actionlib_msgs
    diagnostic_msgs
    rosplan_dispatch_msgs
)

catkin_package()
