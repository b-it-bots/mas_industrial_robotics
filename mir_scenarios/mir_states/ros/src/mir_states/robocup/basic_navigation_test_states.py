import rospy
import smach


class select_target_pose(smach.State):

    """
    Select the next pose that the robot should visit.

    Input
    -----
    task: BNTTask
        Currently executed task. Its internal state will be changed.

    Output
    ------
    move_base_to: str
        Pose name.
    subtask: tuple
        Selected subtask, contains location, orientation, and break duration.
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['pose_selected',
                                       'unknown_pose',
                                       'no_more_targets'],
                             input_keys=['task'],
                             output_keys=['move_base_to',
                                          'task',
                                          'subtask'])

    def execute(self, userdata):
        try:
            subtask = userdata.task.pop()
        except IndexError:
            return 'no_more_targets'

        location_param = "script_server/base/" + subtask[0]
        orientation_param = "script_server/base_orientations/" + subtask[1]

        if not rospy.has_param(location_param):
            rospy.logerr("Location <<%s>> is not on the parameter server" % subtask[0])
            return 'unknown_pose'

        if not rospy.has_param(orientation_param):
            rospy.logerr("Orientation <<%s>> is not on the parameter server" % subtask[1])
            return 'unknown_pose'

        position = rospy.get_param(location_param)
        orientation = rospy.get_param(orientation_param)

        # modify the pose stored on the server with new orientation
        rospy.set_param("script_server/base/" + subtask[0], [position[0], position[1], orientation])

        rospy.loginfo('Selected position: %s, orientation: %s' % (subtask[0], subtask[1]))
        userdata.move_base_to = subtask[0]
        userdata.subtask = subtask
        return 'pose_selected'


class wait_for_desired_duration(smach.State):

    """
    Do not do anything for the amount of time specified by the current subtask.

    Input
    -----
    subtask: tuple
        Currently executed subtask.
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded'],
                             input_keys=['subtask'])

    def execute(self, userdata):
        timeout = userdata.subtask[2]
        rospy.loginfo('Waiting for %s seconds...' % timeout)
        rospy.sleep(int(timeout))
        return 'succeeded'
