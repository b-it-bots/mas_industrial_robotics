#!/usr/bin/env python
#-*- encoding: utf-8 -*-
"""
Add boxes to planning scene to represent the workspace walls.
#TODO this should be replaced in the future with the octomap
Currently planning with octomap is too slow.
"""

__author__ = 'moriarty'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import moveit_msgs.msg
import tf

class ArmWorkspaceRestricter(object):
    """
    """
    def __init__(self):
        # params
        self.event_in = None
        self.is_restricted = False

        # node cycle rate (in seconds)
        self.cycle_time = rospy.get_param('~cycle_time', 0.1)
        self.wall_frame_id = rospy.get_param('~wall_frame_id', "/base_link")
        self.wall_height = rospy.get_param('~wall_height', 0.35)
        self.wall_distance = rospy.get_param('~wall_distance', 0.35)
        # distance to platform from base wall_frame_id (m)
        self.platform_distance = rospy.get_param('~platform_distance', 0.6)
        # height of platform with respect to wall_frame_id (m)
        self.platform_height = rospy.get_param('~platform_height', 0.05)

        # publishers
        self.planning_scene_diff_publisher = rospy.Publisher('~planning_scene',
            moveit_msgs.msg.PlanningScene)

        # subscriber
        rospy.Subscriber('~event_in',
            std_msgs.msg.String,
            self.event_in_cb)

    def event_in_cb(self, msg):
        """
        Obtains an event for the ArmWorkspaceRestricter.

        supported events: "e_start", "e_stop"
        """
        self.event_in = msg.data

    def start(self):
        """
        Starts the Workspace Restricter.

        """
        rospy.loginfo("Ready to start...")
        state = 'INIT'

        while not rospy.is_shutdown():

            if state == 'INIT':
                state = self.init_state()
            elif state == 'IDLE':
                state = self.idle_state()
            elif state == 'RUNNING':
                state = self.running_state()

            rospy.logdebug("State: {0}".format(state))
            rospy.sleep(self.cycle_time)

    def init_state(self):
        """
        Executes the INIT state of the state machine.

        :return: The updated state.
        :rtype: str

        This state waits for moveit to be up.
        """
        if self.planning_scene_diff_publisher.get_num_connections() < 1:
            return 'INIT'
        else:
            return 'IDLE'

    def idle_state(self):
        """
        Executes the IDLE state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event_in == 'e_start':
            return 'RUNNING'
        elif self.event_in == 'e_stop':
            return 'INIT'
        else:
            return 'IDLE'

    def running_state(self):
        """
        Executes the RUNNING state of the state machine.

        :return: The updated state.
        :rtype: str

        """
        if self.event_in == 'e_stop':
            self.remove_walls()
            return 'INIT'
        else:
            if not self.is_restricted:
                self.add_walls()
            return 'RUNNING'

    def add_walls(self):
        self.add_box("restricter_left_wall",
            0.25, self.wall_distance, self.wall_height / 2.0,
            1.0, 0.04, self.wall_height)
        self.add_box("restricter_right_wall",
            0.25, -self.wall_distance, self.wall_height / 2.0,
            1.0, 0.04, self.wall_height)
        self.add_box("restricter_platform",
            self.platform_distance, 0.0, 0.0,
            0.5 , self.wall_distance * 2.0, self.platform_height / 2.0)
        self.is_restricted = True

    def remove_walls(self):
        self.remove_box("restricter_left_wall")
        self.remove_box("restricter_right_wall")
        self.is_restricted = False

    def add_box(self, name, x, y, z, dx, dy, dz, yaw=0):
        """
        Adds two boxes to represent the walls
        """
        rospy.loginfo("Adding walls to moveit scene")

        box_object = moveit_msgs.msg.CollisionObject();
        box_object.header.frame_id = self.wall_frame_id
        box_object.id = name
        box_pose = geometry_msgs.msg.Pose()
        quat = tf.transformations.quaternion_from_euler(yaw, 0, 0)
        box_pose.orientation.w = quat[0]
        box_pose.orientation.x = quat[1]
        box_pose.orientation.y = quat[2]
        box_pose.orientation.z = quat[3]
        box_pose.position.x = x
        box_pose.position.y = y
        box_pose.position.z = z
        box_shape = shape_msgs.msg.SolidPrimitive()
        box_shape.type = box_shape.BOX
        box_shape.dimensions.append(dx)
        box_shape.dimensions.append(dy)
        box_shape.dimensions.append(dz)
        box_object.primitives.append(box_shape)
        box_object.primitive_poses.append(box_pose)
        box_object.operation = box_object.ADD

        planning_scene = moveit_msgs.msg.PlanningScene()
        planning_scene.world.collision_objects.append(box_object)
        planning_scene.is_diff = True

        self.planning_scene_diff_publisher.publish(planning_scene)

    def remove_box(self, name):
        """
        removes name from planning_scene
        """
        rospy.loginfo("removing wall from moveit scene")

        box_object = moveit_msgs.msg.CollisionObject();
        box_object.header.frame_id = self.wall_frame_id
        box_object.id = name
        
        box_object.operation = box_object.REMOVE
        
        planning_scene = moveit_msgs.msg.PlanningScene()
        planning_scene.world.collision_objects.append(box_object)
        planning_scene.is_diff = True

        self.planning_scene_diff_publisher.publish(planning_scene)


def main():
    """
    Listens to event_in.
    When recieves "e_start", and walls haven't been added, 
    they are added to planning scene.
    When recieves "e_stop" removes the walls which were added.
    """
    rospy.init_node("arm_workspace_restricter", anonymous=True)
    workspace_restricter = ArmWorkspaceRestricter()
    workspace_restricter.start()

if __name__ == '__main__':
    main()
