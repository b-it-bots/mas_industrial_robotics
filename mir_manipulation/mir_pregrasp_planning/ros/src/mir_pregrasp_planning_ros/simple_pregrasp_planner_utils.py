#!/usr/bin/env python
"""
This module contains functions used by the simple_pregrasp_planner node.

"""
# -*- encoding: utf-8 -*-

import math
import numpy
import copy
import tf.transformations as transformations
from geometry_msgs.msg import Quaternion


def modify_pose(pose_in, height_threshold, standing_angle=270., angular_tolerance=2.):
    """
    Computes a modified pose (of an object) only if the object is
    standing (vertically) on a surface, based on the following criteria:

    - If the object's height is below the height_threshold, then a modified\
      pose is computed consisting of a rotation of 90 degrees of pose_in\
      (e.g. the new pose will have the object laying horizontally).
    - If the object's height is not below the height_threshold, then the\
      orientation is preserved (i.e. still standing), but the rotation\
      will be set to zero degrees around the X axis (assuming the X axis\
      is pointing upwards, see module description of 'simple_grasp_planner').

    It also returns whether the object is standing.

    :param pose_in: The pose to be modified if a set of criteria are met.
    :type pose_in: geometry_msgs.msg.PoseStamped

    :param height_threshold: If the object is standing and its height
        does not exceed this threshold, a rotated pose is then returned.
    :type height_threshold: float

    :param standing_angle: The angle (in degrees) used to check if
        the pose has its X axis pointing upwards.
    :type standing_angle: float

    :param angular_tolerance: The tolerance (in degrees) used to check if
        the pose has its X axis pointing upwards.
    :type angular_tolerance: float

    :return: The modified pose and whether the object is standing.
    :rtype: brics_actuator.msg.PoseStamped, bool

    """
    standing = False
    pose_out = copy.deepcopy(pose_in)

    # Checking if the orientation is 'standing':
    # if the pose's X axis of the pose is pointing upwards,
    # then it has a rotation of 'standing_angle' degrees around the Y axis
    roll, pitch, yaw = transformations.euler_from_quaternion([
        pose_in.pose.orientation.x, pose_in.pose.orientation.y,
        pose_in.pose.orientation.z, pose_in.pose.orientation.w
    ])

    # to have the range of angles from 0-360
    decision_angle = math.degrees(pitch) % 360.0

    if (standing_angle - angular_tolerance) <= decision_angle \
            <= (standing_angle + angular_tolerance):
        standing = True
    # in case the object is upside down
    if (standing_angle - angular_tolerance) <= (decision_angle + 180) \
            <= (standing_angle + angular_tolerance):
        standing = True

    if standing:
        if pose_in.pose.position.z < height_threshold:
            # The object is re-oriented to be laying horizontally,
            # but maintaining its rotation
            new_orientation = transformations.quaternion_from_euler(
                roll, pitch - (math.pi / 2), yaw
            )
            pose_out.pose.orientation.x = new_orientation[0]
            pose_out.pose.orientation.y = new_orientation[1]
            pose_out.pose.orientation.z = new_orientation[2]
            pose_out.pose.orientation.w = new_orientation[3]
            standing = False
        else:
            # Retain standing orientation, but with no rotation around the X axis
            pose_out.pose.orientation.x = 0.0
            pose_out.pose.orientation.y = math.cos(math.pi / 4)
            pose_out.pose.orientation.z = 0.0
            pose_out.pose.orientation.w = -math.cos(math.pi / 4)
    else: # flatten the pose so that it is parallel to the ground
        new_orientation = transformations.quaternion_from_euler(0.0, 0.0, yaw)
        pose_out.pose.orientation = Quaternion(*new_orientation)

    return pose_out, standing


def modify_pose_rotation(pose, offset=0.0, reference_axis='z', rotation_range=None):
    """
    Modifies the orientation of a pose, for a single rotation axis (reference_axis),
    by adding an offset and limiting the rotation to be within certain range. If no
    rotation_range is specified, the pose is not modified (except for the specified
    offset).

    :param pose: The pose to be modified.
    :type pose: geometry_msgs.msg.PoseStamped

    :param offset: Rotation offset to add to the reference_axis of the pose (in degrees).
    :type offset: float

    :param reference_axis: The rotation axis of the pose to be modified (e.g. x, y, z).
    :type reference_axis: str

    :param rotation_range: If specified, the rotation will be limited until the
        rotation_range value (in degrees). For instance, if rotation_range is set
        to [0, 180], where 0 is the minimum and 180 degrees is the maximum, the
        rotation will remain in half circle (e.g. this parameter can be helpful
        for symmetric objects, since there a 180 degree rotation makes no difference).
    :type rotation_range: list

    :return: The modified pose.
    :rtype: geometry_msgs.msg.PoseStamped

    """
    pose_out = copy.deepcopy(pose)
    orientation_in = (
        pose_out.pose.orientation.x, pose_out.pose.orientation.y,
        pose_out.pose.orientation.z, pose_out.pose.orientation.w
    )

    angles_in = transformations.euler_from_quaternion(orientation_in)
    # Add 360 degrees to the negative angles, since the range of
    # euler_from_quaternion is -PI to PI (-180 to 180 degrees)
    angles_in = [(math.pi * 2) + angle if angle < 0.0 else angle for angle in angles_in]

    euler_angles = {
        'x': angles_in[0],
        'y': angles_in[1],
        'z': angles_in[2],
    }

    # Ensure the offset is not more than 360 degrees
    offset %= 360
    target_angle = euler_angles[reference_axis] + math.radians(offset)

    if rotation_range is not None:
        target_angle = restrict_angle_to_range(
            target_angle, math.radians(offset), list(numpy.radians(rotation_range))
        )

    euler_angles[reference_axis] = target_angle
    angles_out = [euler_angles['x'], euler_angles['y'], euler_angles['z']]

    orientation_out = transformations.quaternion_from_euler(
        angles_out[0], angles_out[1], angles_out[2]
    )

    pose_out.pose.orientation.x = orientation_out[0]
    pose_out.pose.orientation.y = orientation_out[1]
    pose_out.pose.orientation.z = orientation_out[2]
    pose_out.pose.orientation.w = orientation_out[3]

    return pose_out


def restrict_angle_to_range(angle, offset, rotation_range):
    """
    Restricts the value of an angle to be within the a range of a
    circle (with the offset added to this range).
    The rotation range might be specified with its first value (e.g. minimum)
    greater than the second value (e.g. maximum) to cover a range within the
    circle that passes through zero.

    :param angle: The angle to restrict (in radians).
    :type angle: float

    :param offset: Rotation offset to add to the angle (in radians).
    :type offset: float

    :param rotation_range: The range in which the angle is allowed to be (in radians).
        The first value represents the minimum and the second value the maximum.
        Note that the first value is allowed to be greater than the second.
    :type rotation_range: list

    :return: The angle restricted to a range within a circle (in radians).
    :rtype: float

    :Examples:

    >>> angle = math.radians(60)
    >>> offset = math.radians(0)
    >>> rotation_range = list(numpy.radians([0, 180]))
    >>> restrict_angle_to_range(angle, offset, rotation_range)
    1.0472      # 60 degrees

    >>> angle = math.radians(60)
    >>> offset = math.radians(0)
    >>> rotation_range = list(numpy.radians([90, 270]))
    >>> restrict_angle_to_range(angle, offset, rotation_range)
    4.1888      # 240 degrees

    >>> angle = math.radians(60)
    >>> offset = math.radians(100)
    >>> rotation_range = list(numpy.radians([0, 180]))
    >>> restrict_angle_to_range(angle, offset, rotation_range)
     4.1888    # 160 degrees

    >>> angle = math.radians(100)
    >>> offset = math.radians(0)
    >>> # note that the first value is greater than the second value
    >>> rotation_range = list(numpy.radians([270, 90]))
    >>> restrict_angle_to_range(angle, offset, rotation_range)
     4.8869    # 280 degrees

    """
    inverted_range = (rotation_range[0] % math.radians(360)) > \
                     (rotation_range[1] % math.radians(360))
    if inverted_range:
        if (offset + rotation_range[1]) < angle < (offset + rotation_range[0]):
            angle += math.radians(180.0)
    else:
        if not (offset + rotation_range[0] <= angle <= (offset + rotation_range[1])):
            angle += math.radians(180.0)

    # Ensure the angle will not be greater than 360 degrees
    return angle % math.radians(360)
