from __future__ import print_function

import tf
import os
import math
import yaml
import rospy

from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker

class Utils(object):

    """Utils class to create marker based on a config file as requested."""

    def __init__(self):
        # read ros params
        self._global_frame = rospy.get_param('~global_frame', 'map')
        self._base_link_to_ws_edge = rospy.get_param('~base_link_to_ws_edge', 0.4)
        self._alpha = rospy.get_param('~alpha', 1.0)

        self._model_path = rospy.get_param('~model_path', None)
        if self._model_path is None:
            raise Exception('Model path not provided.')
        if not os.path.exists(self._model_path):
            raise Exception('Invalid model path provided. ' + str(self._model_path) + ' does not exist')

        marker_config_file = rospy.get_param('~model_to_marker_config', None)
        self.marker_config = None
        with open(marker_config_file) as file_obj:
            self.marker_config = yaml.safe_load(file_obj)
        if self.marker_config is None:
            raise Exception('Model config not provided.')

        navigation_goals = rospy.get_param('~navigation_goals', None)
        if navigation_goals is None:
            raise Exception('Navigation goal file not provided.')

        # class variables
        self.marker_counter = 0
        self.ws_pose = dict()
        # offset nav goal based on size of robot to initialise workstation poses
        for ws, pos in navigation_goals.iteritems():
            delta_x = (math.cos(pos[2]) * self._base_link_to_ws_edge)
            delta_y = (math.sin(pos[2]) * self._base_link_to_ws_edge)
            self.ws_pose[ws.lower()] = [pos[0] + delta_x, pos[1] + delta_y, pos[2]]

    def get_markers_from_ws_pos(self):
        """Create markers for workstations

        :returns: list of visualization_msgs.Marker

        """
        markers = []
        for ws, pos in self.ws_pose.iteritems():
            if 'ws' in ws:
                marker = self.get_marker_from_obj_name_and_pos('ws', x=pos[0],
                                                               y=pos[1], yaw=pos[2])
            elif 'sh' in ws:
                marker = self.get_marker_from_obj_name_and_pos('sh', x=pos[0],
                                                               y=pos[1], yaw=pos[2])
            elif 'pp' in ws:
                marker = self.get_marker_from_obj_name_and_pos('pp', x=pos[0],
                                                               y=pos[1], yaw=pos[2])
            elif 'cb' in ws:
                marker = self.get_marker_from_obj_name_and_pos('cb', x=pos[0],
                                                               y=pos[1], yaw=pos[2])
                config = self.marker_config['cb']
                marker.type = Marker.CYLINDER
                marker.scale.x = marker.scale.y = config['scale']
                marker.scale.z = 0.02
            else:
                continue
            markers.append(marker)
        return markers

    def get_markers_for_youbot(self, robot_ws='start'):
        """Create markers for robot

        :ws_name: str
        :returns: list of visualization_msgs.Marker

        """
        x, y, yaw = self.ws_pose.get(robot_ws.lower(), [0.0, 0.0, 0.0])
        delta_x = (math.cos(yaw) * self._base_link_to_ws_edge)
        delta_y = (math.sin(yaw) * self._base_link_to_ws_edge)
        x, y = x-delta_x, y-delta_y
        keys = ['youbot', 'arm_0', 'arm_1', 'arm_2', 'arm_3', 'arm_4', 'arm_5',\
                'arm_palm', 'youbot_plate', 'yb_wheel_lf', 'yb_wheel_rf',\
                'yb_wheel_lb', 'yb_wheel_rb']
        marker_dict = {key:self.get_marker_from_obj_name_and_pos(key, x=x, y=y, z=0.05, yaw=yaw)
                       for key in keys}
        for key, marker in marker_dict.iteritems():
            if 'wheel' in key:
                marker.type = Marker.SPHERE
        return marker_dict.values()

    def get_markers_from_obj_on_robot(self, obj, platform, robot_ws):
        """Create markers for objects stored on robot's platform

        :obj: str
        :platform: str
        :robot_ws: str
        :returns: list of visualization_msgs.Marker

        """
        x, y, yaw = self.ws_pose.get(robot_ws.lower(), [0.0, 0.0, 0.0])
        delta_x = (math.cos(yaw) * self._base_link_to_ws_edge)
        delta_y = (math.sin(yaw) * self._base_link_to_ws_edge)
        x, y = x-delta_x, y-delta_y
        obj_pose_offsets = {platform:(-0.1, y) for platform, y in zip(
                                ['platform_middle', 'platform_left', 'platform_right'],
                                [0.0, 0.1, -0.1])}

        offset = obj_pose_offsets.get(platform.lower(), None)
        if offset:
            delta_x = (math.cos(yaw) * offset[0]) + (-math.sin(yaw) * offset[1])
            delta_y = (math.sin(yaw) * offset[0]) + (math.cos(yaw) * offset[1])
            marker = self.get_marker_from_obj_name_and_pos(
                    obj, x=x+delta_x, y=y+delta_y, z=0.11, yaw=yaw)
            return marker
        else:
            rospy.logwarn('Object ' + obj + ' on unknown platform '+ platform)
        
    def get_markers_from_obj_on_ws(self, obj_list, ws_name, container_to_obj=None):
        """Create markers for objects on a given workstation

        :obj_list: list
        :ws_name: str
        :container_to_obj: dict
        :returns: list of visualization_msgs.Marker

        """
        ws = ws_name.lower()
        pos = self.ws_pose.get(ws, None)
        if pos is None:
            return []
        markers = []
        if 'ws' in ws or 'sh' in ws:
            obj_pose_offsets = [(x, y) for x in [0.05, 0.2]
                                       for y in [0.0, 0.15, -0.15, 0.3, -0.3]]
        elif 'cb' in ws:
            cb_radius = self.marker_config['cb']['scale']/2.0
            obj_pose_offsets = [(math.cos(math.radians(theta))*cb_radius*0.8 + cb_radius,
                                 math.sin(math.radians(theta))*cb_radius*0.8)
                                for theta in range(0, 360, 36)]
        else:
            return []

        if len(obj_list) > 10:
            rospy.logwarn('Only showing 10 objects on ' + str(ws))
        for obj, offset in zip(obj_list, obj_pose_offsets):
            delta_x = (math.cos(pos[2]) * offset[0]) + (-math.sin(pos[2]) * offset[1])
            delta_y = (math.sin(pos[2]) * offset[0]) + (math.cos(pos[2]) * offset[1])
            marker = self.get_marker_from_obj_name_and_pos(
                    obj, x=pos[0]+delta_x, y=pos[1]+delta_y, z=0.1, yaw=pos[2])
            markers.append(marker)
            # for containers
            if container_to_obj and obj in container_to_obj:
                objs_in_container = container_to_obj[obj]
                delta_x_container = (math.cos(pos[2]) * (offset[0]+0.05)) \
                                    + (-math.sin(pos[2]) * offset[1])
                delta_y_container = (math.sin(pos[2]) * (offset[0]+0.05)) \
                                    + (math.cos(pos[2]) * offset[1])
                z = 0.1
                for obj_in_container in objs_in_container:
                    z += 0.05
                    marker = self.get_marker_from_obj_name_and_pos(
                            obj_in_container,
                            x=pos[0]+delta_x_container,
                            y=pos[1]+delta_y_container,
                            z=z, yaw=pos[2])
                    markers.append(marker)
        return markers

    def get_marker_from_obj_name_and_pos(self, obj_name, x=0.0, y=0.0, z=0.0,
                                          yaw=0.0, frame=None):
        """Create a marker for a given object at a given position

        The marker is created in such a way that the position (given by x, y and z)
        lies at the middle bottom of the model.

        If the given object name cannot be mapped to a model, a default marker 
        is used instead. Default marker is a small red cube.

        :obj_name: string
        :x: float
        :y: float
        :z: float
        :yaw: float
        :frame: str
        :returns: visualization_msgs.Marker

        """
        if frame is None:
            frame = self._global_frame

        obj_name = obj_name.split('-')[0] if '-' in obj_name else obj_name
        obj_name = obj_name.lower()
        marker = Marker()
        if obj_name not in self.marker_config:
            rospy.logwarn('Could not find ' + str(obj_name) + '. Using default marker')
            marker.type = Marker.CUBE
            config = self.marker_config['default']
        else:
            config = self.marker_config[obj_name]
            file_path = os.path.join(self._model_path, config['file_name'])
            if os.path.exists(file_path):
                resource_file = 'file://' + file_path
                marker.type=Marker.MESH_RESOURCE
                marker.mesh_resource = resource_file
            else:
                rospy.logwarn('Could not find file ' + str(file_path) + '. Using default marker')
                marker.type = Marker.CUBE
                config = self.marker_config['default']

        marker.header.stamp = rospy.Time.now()

        marker.scale.x = marker.scale.y = marker.scale.z = config['scale']

        marker.color.r = config['color']['r']
        marker.color.g = config['color']['g']
        marker.color.b = config['color']['b']
        marker.color.a = self._alpha

        angle = math.radians(config['offset']['yaw']) + yaw
        quat = tf.transformations.quaternion_from_euler(
                math.radians(config['offset']['roll']),
                math.radians(config['offset']['pitch']),
                angle)
        marker.pose.orientation = Quaternion(*quat)
        delta_x = (math.cos(angle) * config['offset']['x']) \
                + (-math.sin(angle) * config['offset']['y'])
        delta_y = (math.sin(angle) * config['offset']['x']) \
                + (math.cos(angle) * config['offset']['y'])
        marker.pose.position.x = x + delta_x
        marker.pose.position.y = y + delta_y
        marker.pose.position.z = z + config['offset']['z']

        marker.header.frame_id = frame
        self.marker_counter += 1
        marker.id = self.marker_counter
        return marker

