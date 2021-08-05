#!/usr/bin/python
import geometry_msgs.msg
import rospy
import tf


def get_orientation_from_param_server(
    orientation_goal, frame_id="map", param_ns="/script_server/base_orientations/"
):
    assert type(orientation_goal) == str, "'orientation_goal' must be a string."

    parameter_name = param_ns + orientation_goal
    if not rospy.has_param(parameter_name):
        rospy.logerr(
            "parameter '{}' does not exist on ROS Parameter Server, aborting...".format(
                parameter_name
            )
        )
        return None
    else:
        angle = rospy.get_param(parameter_name)

        quat = tf.transformations.quaternion_from_euler(0, 0, angle)
        orientation = geometry_msgs.msg.Quaternion()
        orientation.x = quat[0]
        orientation.y = quat[1]
        orientation.z = quat[2]
        orientation.w = quat[3]

        return orientation


def get_pose_from_param_server(
    navigation_goal, frame_id="map", param_ns="/script_server/base/"
):
    """
    param navigation_goal: the name of the navigation goal to obtain the pose.
    type navigation_goal: str

    param frame_id: the name of the reference frame for the pose.
    type frame_id: str

    param param_ns: the namespace of the parameter server.
    type param_ns: str

    return: the pose specified by name; or None if the pose name is not
        in the param server.
    return type: geometry_msgs.msg.PoseStamped or None

    """
    assert type(navigation_goal) == str, "'navigation_goal' must be a string."

    parameter_name = param_ns + navigation_goal
    if not rospy.has_param(parameter_name):
        rospy.logerr(
            "parameter '{}' does not exist on ROS Parameter Server, aborting...".format(
                parameter_name
            )
        )
        return None
    else:
        pose = geometry_msgs.msg.PoseStamped()
        pose_2d = rospy.get_param(parameter_name)

        # TODO: Oscar fix this... No Jose i will not fix it
        # pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id
        pose.pose.position.x = pose_2d[0]
        pose.pose.position.y = pose_2d[1]
        pose.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0, 0, pose_2d[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose


if __name__ == "__main__":
    import sys

    while True:
        nav_goal = raw_input(
            "Please specify the name of the navigation_goal ('q' to quit): "
        )
        if nav_goal == "q":
            print("Quitting...")
            sys.exit(0)
        pose = get_pose_from_param_server(nav_goal)
        print("Pose for navigation goal '{}': {}".format(nav_goal, pose))
