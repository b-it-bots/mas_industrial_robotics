from __future__ import print_function

import rospy
import yaml
from mir_planning_msgs.msg import (GenericExecuteActionGoal,
                                   GenericExecuteActionResult,
                                   PlanActionResult)
from mir_planning_visualisation.utils import Utils
from visualization_msgs.msg import Marker, MarkerArray


class PlanVisualiser(object):
    """ Class to visualize tasks in plan
    such as pick, unstage and move base
    """

    def __init__(self):
        """ Class constructor to create
        necessary publisher and subscriber
        """

        self._utils = Utils()
        marker_config_file = rospy.get_param(
            '~plan_marker_color_config',
            None
        )
        self.plan_marker_config = None
        with open(marker_config_file) as file_obj:
            self.plan_marker_config = yaml.safe_load(file_obj)
        if self.plan_marker_config is None:
            raise Exception('Model config not provided.')
        self._complete_plan = None
        self._prev_location = "START"

    def _create_subscriber_to_task_planner_server(self):

        rospy.Subscriber(
            "/mir_task_planning/task_planner_server/plan_task/result",
            PlanActionResult,
            self._task_planner_cb
        )

    def _task_planner_cb(self, msg):

        if msg:
            self._complete_plan = msg.result.plan.plan
            self._plan_changed = True

    def _set_markers_for_complete_plan(self, kb_markers, kb_data):

        move_base_config = self.plan_marker_config['move_base']
        pick_config = self.plan_marker_config['pick']
        move_base_markers = []
        curr_location = kb_data['robot_ws'].upper()
        if curr_location != self._prev_location:
            self._prev_location = curr_location
            for idx, action in enumerate(self._complete_plan):
                if (action.name.upper() == "MOVE_BASE" and
                    action.parameters[2].value.upper() == kb_data['robot_ws']):
                    self._complete_plan = self._complete_plan[idx+1:]
                    break

        for action in self._complete_plan:
            if action.name.upper() == "MOVE_BASE":
                source = action.parameters[1].value.lower()
                destination = action.parameters[2].value.lower()
                source_loc = self._utils.ws_pose[source]
                dest_loc = self._utils.ws_pose[destination]
                marker = self._utils.get_arc_marker(
                    source_loc,
                    dest_loc
                )
                marker.color.r = move_base_config['color']['r']
                marker.color.g = move_base_config['color']['g']
                marker.color.b = move_base_config['color']['b']
                marker.color.a = move_base_config['color']['a']
                move_base_markers.append(marker)
            elif action.name.upper() == "PICK":
                for marker in kb_markers[0]:
                    if marker.text.upper() == action.parameters[2].value:
                        marker.mesh_use_embedded_materials = True
                        marker.color.r = pick_config['color']['r']
                        marker.color.g = pick_config['color']['g']
                        marker.color.b = pick_config['color']['b']
                        marker.color.a = pick_config['color']['a']
                        marker.scale.x = marker.scale.y = marker.scale.z = pick_config['scale']['x']
                        break

        kb_markers.append(move_base_markers)

    def visualise(self, kb_markers, kb_data=None):
        """
        TODO: docstring for visualise method

        :kb_markers: list of lists
        :kb_data: dict
        :returns: list of visualization_msgs.Marker

        """
        if kb_data is None:
            return None

        self._utils.marker_counter = 10000 # to avoid marker id repetition
        self._create_subscriber_to_task_planner_server()
        if self._complete_plan:
            self._set_markers_for_complete_plan(kb_markers, kb_data)

        return sum(kb_markers, [])
