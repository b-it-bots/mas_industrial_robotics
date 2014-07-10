#!/usr/bin/env python
#-*- encoding: utf-8 -*-
"""
Adds an attached object to the gripper in the planning scene.

The node receives a list of objects and the name of an object to select
from this list. The first object in the list with the matching name is taken.
A box with the dimensions defined in the selected object is attached
to the gripper. The box is attached to the arm at attachment_frame_id

event_in expects "e_start" and "e_stop",
start attaches the object, stop detaches the object.
"""

__author__ = 'moriarty'

import rospy
import std_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import moveit_msgs.msg
import mcr_perception_msgs.msg
import tf

class GraspedObjectAttacher(object):
    """
    """
    def __init__(self):
        # params
        self.event_in = None
        self.object_list = None
        self.object_name = None
        self.is_attached = False # Set in attach_box, detach_box
        self.attached_object_name = None # set in attach_box, detach_box
        self.listener = tf.TransformListener()

        # node cycle rate (in seconds)
        self.cycle_time = rospy.get_param('~cycle_time', 0.1)
        self.attachment_frame_id = rospy.get_param('~attachment_frame_id',
            "grasp_link")
        self.fixed_frame_id = rospy.get_param('~fixed_frame_id',
            "base_link")

        # publishers
        self.planning_scene_diff_publisher = rospy.Publisher('/planning_scene',
            moveit_msgs.msg.PlanningScene, latch=True)

        # subscriber
        rospy.Subscriber('~event_in', std_msgs.msg.String, self.event_in_cb)
        rospy.Subscriber('~object_list', mcr_perception_msgs.msg.ObjectList,
            self.object_list_cb)
        rospy.Subscriber('~object_name', std_msgs.msg.String,
            self.object_name_cb)

    def event_in_cb(self, msg):
        """
        Obtains an event for the GraspedObjectAttacher.

        supported events: "e_start", "e_stop"
        """
        self.event_in = msg.data
        rospy.logdebug("recieved event in: {0}".format(self.event_in))

    def object_list_cb(self, msg):
        """
        Obtains a list of objects on the workspace

        """
        self.object_list = msg
        rospy.logdebug("recieved object list")

    def object_name_cb(self, msg):
        """
        Obtains the name of the object to be attached

        """
        self.object_name = msg.data
        rospy.logdebug("got object name: {0}".format(self.object_name))

    def start(self):
        """
        Starts the Grasped Object Attacher.

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
        elif not self.object_name:
            return 'INIT'
        elif not self.object_list:
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
            self.object_name = None
            self.object_list = None
            self.event_in = None
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
            self.object_name = None
            self.object_list = None
            self.event_in = None
            self.detach_object()
            return 'INIT'
        else:
            if not self.is_attached:
                self.attach_object()
            return 'RUNNING'

    def attach_object(self):
        tf_time = self.listener.getLatestCommonTime(self.attachment_frame_id,
            self.fixed_frame_id)
        trans, rot = self.listener.lookupTransform(self.fixed_frame_id,
            self.attachment_frame_id, tf_time)
        pitch = tf.transformations.euler_from_quaternion(rot)[1]
        # Look up object_name from list
        for obj in self.object_list.objects:
            if obj.name == self.object_name:
                self.attach_box(self.object_name,
                    obj.dimensions.vector.y,
                    obj.dimensions.vector.x,
                    obj.dimensions.vector.z,
                    pitch)
                break
        else:
            rospy.logwarn("Object to attach was not in object list")

    def detach_object(self):
        self.detach_box(self.attached_object_name)

    def attach_box(self, name, dx, dy, dz, pitch):
        """
        attaches a box to the robot at the attachment frame
        """
        rospy.loginfo("Attaching object to planning scene robot")
        self.attached_object_name = "attached_" + name

        box_object = moveit_msgs.msg.AttachedCollisionObject();
        box_object.link_name = self.attachment_frame_id
        box_object.object.header.frame_id = self.attachment_frame_id
        box_object.object.id = self.attached_object_name
        box_pose = geometry_msgs.msg.Pose()
        quat = tf.transformations.quaternion_from_euler(0, pitch, 0)
        box_pose.orientation.w = quat[0]
        box_pose.orientation.x = quat[1]
        box_pose.orientation.y = quat[2]
        box_pose.orientation.z = quat[3]
        box_shape = shape_msgs.msg.SolidPrimitive()
        box_shape.type = box_shape.BOX
        box_shape.dimensions.append(dx)
        box_shape.dimensions.append(dy)
        box_shape.dimensions.append(dz)
        box_object.object.primitives.append(box_shape)
        box_object.object.primitive_poses.append(box_pose)
        box_object.object.operation = box_object.object.ADD

        planning_scene = moveit_msgs.msg.PlanningScene()
        planning_scene.robot_state.attached_collision_objects.append(box_object)
        planning_scene.is_diff = True
        self.planning_scene_diff_publisher.publish(planning_scene)

        self.is_attached = True

    def detach_box(self, name):
        """
        detaches "name" from robot
        """
        rospy.loginfo("detaching object from planning scene robot")

        box_object = moveit_msgs.msg.AttachedCollisionObject();
        box_object.link_name = self.attachment_frame_id
        box_object.object.header.frame_id = self.attachment_frame_id
        box_object.object.id = name
        box_object.object.operation = box_object.object.REMOVE
        
        planning_scene = moveit_msgs.msg.PlanningScene()
        planning_scene.robot_state.attached_collision_objects.append(box_object)
        planning_scene.is_diff = True
        self.planning_scene_diff_publisher.publish(planning_scene)

        self.is_attached = False
        self.attached_object_name = None

def main():
    """
    Listens to event_in, object_list, object_name

    Requires object_list and object_name before recieving "e_start" on event_in.
    e_start: Attaches first object_name from object_list to attachment_frame_id.
    e_stop: Detaches attached object.
    """
    rospy.init_node("grasped_object_attacher", anonymous=True)
    object_attacher = GraspedObjectAttacher()
    object_attacher.start()


if __name__ == '__main__':
    main()
