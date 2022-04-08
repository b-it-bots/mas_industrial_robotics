#!/usr/bin/env python
"""
This component publishes the trajetory passing through given way points!.

**Input(s):**
  * `waypoint_list`: Way points through which the manipulator will be moved.

**Parameter(s):**
  * `move_group`: MoveIt! interface.
  * `arm`: Name of the group to move.
  * `loop_rate`: Node cycle rate (in hz).
"""
#-*- encoding: utf-8 -*-

import rospy
import actionlib
import moveit_commander
import std_msgs.msg
import moveit_msgs.msg
import brics_actuator.msg
from mcr_manipulation_msgs.msg import JointSpaceWayPointsList
import trajectory_msgs.msg
import numpy as np

from sensor_msgs.msg import JointState

class WaypointTrajectoryGeneration(object):
    """
    Components that move the arm in a planned motion.

    """
    def __init__(self):
        # Params
        self.event = None
        self.waypoint_list = None

        # MoveIt! interface
        move_group = rospy.get_param('~move_group', None)
        assert move_group is not None, "Move group must be specified."

        # Wait for MoveIt!
        client = actionlib.SimpleActionClient(move_group, moveit_msgs.msg.MoveGroupAction)
        rospy.loginfo("Waiting for '{0}' server".format(move_group))
        client.wait_for_server()
        rospy.loginfo("Found server '{0}'".format(move_group))

        # Name of the group to move
        arm = rospy.get_param('~arm', None)
        assert arm is not None, "The group to be moved must be specified (e.g. arm)."

        # Set up MoveIt!
        self.arm = moveit_commander.MoveGroupCommander(arm)
        self.robot = moveit_commander.RobotCommander()

        # Whether MoveIt! should wait for the arm to be stopped.
        self.wait_for_motion = rospy.get_param('~wait_for_motion', True)

        # Node cycle rate (in hz)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10))

        # Publishers
        self.event_out = rospy.Publisher("~event_out", std_msgs.msg.String, queue_size=1)
        self.trajectory_pub = rospy.Publisher("~waypoint_trajectory", \
            trajectory_msgs.msg.JointTrajectory, queue_size=1)
        # Subscribers
        rospy.Subscriber("~event_in", std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber(
            "waypoint_list", JointSpaceWayPointsList,
            self.waypoint_list_cb
        )

    def waypoint_list_cb(self, msg):
        """
        Obtains the way points that the joint will be going through.

        """
        self.waypoint_list = msg

    def event_in_cb(self, msg):
        """
        Starts a planned motion based on the specified arm position.

        """
        self.event = msg.data

    def start(self):
        """
        Starts the component.

        """
        rospy.loginfo("Ready to start now...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            self.loop_rate.sleep()

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_start':
            return 'IDLE'
        elif self.event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            return 'INIT'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """

        if self.event == 'e_stop':
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        elif self.waypoint_list:
            return 'RUNNING'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event == 'e_stop':
            self.arm.stop()
            self.reset_component_data()
            self.event_out.publish('e_stopped')
            return 'INIT'
        else:
            move_status = self.prepare_and_pub_plan()

            if move_status:
                self.event_out.publish('e_success')
            else:
                self.event_out.publish('e_failure')

        self.reset_component_data()
        return 'INIT'
 
    def brics_joint_positions_to_list(self, joint_configuration):
        """
        Converts a BRICS message of joint positions into a list of real values.

        :param joint_configuration: Joint configuration as a BRICS message.
        :type joint_configuration: brics_actuator.msg.JointPositions

        :return: A list of real values representing the joints as specified
            by the BRICS message.
        :rtype: list

        """
        return [joint.value for joint in joint_configuration.positions]

    def reset_component_data(self):
        """
        Clears the data of the component.

        """
        self.event = None
        self.waypoint_list = None

    def set_joint_current_state(self, robot_state, last_plan_joint_val, last_plan_joint_names):

        last_plan_joint_names = np.array(last_plan_joint_names)
        temp_state_name_array = np.array(robot_state.joint_state.name)
        temp_current_pos = list(robot_state.joint_state.position)
        for idx, name in enumerate(last_plan_joint_names):
            idx_in_robot_state = np.where(temp_state_name_array == name)
            temp_current_pos[idx_in_robot_state[0][0]] = last_plan_joint_val[idx]
        robot_state.joint_state.position = tuple(temp_current_pos)
        return robot_state

    def get_sub_plan(self, waypoints, is_name):
        '''
        Returns appended trajectory through given way points.

        :param waypoints: List of waypoints (Either in the form of joint values of 
            predefined strings).
        :type waypoints : List

        :param is_name: Parameter is true if the waypoints are the list of strings
        :type is_name: Bool

        :return: Concatinated and time corrected robot trajectory.
        :rtype: trajectory_msgs.msg.JointTrajectory
        '''
        sub_plans = []
        '''
           Create subplans for each joint configuration
        ''' 
        current_state = self.robot.get_current_state()
        for idx, config in enumerate(waypoints):
            
            self.arm.set_start_state(current_state)
            if is_name:
                self.arm.set_named_target(config)
                config = None
            else:
                print("\nWaypoint:", config)
                self.arm.set_joint_value_target(list(config))
            joint_variable = JointState()
            joint_variable.position=config
            sub_plan = self.arm.plan(joint_variable)
            if sub_plan[0] == False:
                return None
            else:
                sub_plan = sub_plan[1]
            if len(sub_plan.joint_trajectory.points) == 0:
                return None

            sub_plan.joint_trajectory.points = sub_plan.joint_trajectory.points[1:]
            sub_plans.append(sub_plan)
            current_state = self.set_joint_current_state(current_state, \
                    sub_plan.joint_trajectory.points[-1].positions, \
                    sub_plan.joint_trajectory.joint_names)
        '''
          Combine subplans and adjust time_from_start for each new 
          subplans relative to previous subplan.
        '''
        concatinated_plan = sub_plans[0]
        sub_plans.remove(sub_plans[0])

        for idx, plan in enumerate(sub_plans):
          last_time = concatinated_plan.joint_trajectory.points[-1].time_from_start

          for point in plan.joint_trajectory.points:
              point.time_from_start = last_time + point.time_from_start
              concatinated_plan.joint_trajectory.points.append(point)

        return concatinated_plan
        

    def prepare_and_pub_plan(self):
        '''
        Prepares and publishes the trajectory through given waypoints
        '''
        waypoints = []
        is_name = None
        current_state = self.robot.get_current_state()

        '''
           Get the list of waypoints.
        ''' 
        if len(self.waypoint_list.list_of_joint_values_lists)>0:
            is_name = False
            for idx, values in enumerate(self.waypoint_list.list_of_joint_values_lists):
                waypoints.append(self.waypoint_list.list_of_joint_values_lists[idx].data)
        if len(self.waypoint_list.list_of_joint_positions)>0:
            is_name = True
            for idx, names in enumerate(self.waypoint_list.list_of_joint_positions):
                waypoints.append(self.waypoint_list.list_of_joint_positions[idx].data)     
        concatinated_plan = self.get_sub_plan(waypoints, is_name)
        if concatinated_plan is None:
            return False 

        self.trajectory_pub.publish(concatinated_plan.joint_trajectory)
        self.arm.execute(concatinated_plan, True)
        return True

def main():
    rospy.init_node("waypoint_trajectory_generation", anonymous=True)
    waypoint_trajectory_generation = WaypointTrajectoryGeneration()
    waypoint_trajectory_generation.start()
