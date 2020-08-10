from __future__ import print_function

import rospy
import yaml
from mir_planning_msgs.msg import PlanActionResult
from mir_planning_visualisation.utils import Utils


class PlanVisualiser(object):
    """
    Class to visualize tasks in plan.
    Currently supported actions ``move_base`` and ``pick``
    """

    def __init__(self):
        """ Class constructor to create
        necessary publisher and subscriber
        """

        marker_config_file = rospy.get_param("~plan_marker_color_config", None)

        self._utils = Utils()

        self.plan_marker_config = None
        with open(marker_config_file) as file_obj:
            self.plan_marker_config = yaml.safe_load(file_obj)
        if self.plan_marker_config is None:
            raise Exception("Plan marker config not provided.")

        self._complete_plan = None
        self._prev_location = "START"

        rospy.Subscriber("~plan", PlanActionResult, self._task_planner_cb)

    def _task_planner_cb(self, msg):
        if msg:
            self._complete_plan = msg.result.plan.plan
            self._plan_changed = True

    def _set_markers_for_complete_plan(self, kb_markers, kb_data):
        move_base_config = self.plan_marker_config["move_base"]
        pick_config = self.plan_marker_config["pick"]
        move_base_markers = []
        curr_location = kb_data["robot_ws"].upper()
        if curr_location != self._prev_location:
            self._prev_location = curr_location
            for idx, action in enumerate(self._complete_plan):
                if (
                    action.name.upper() == "MOVE_BASE"
                    and action.parameters[2].value.upper() == kb_data["robot_ws"]
                ):
                    self._complete_plan = self._complete_plan[idx + 1 :]
                    break

        for action in self._complete_plan:
            if action.name.upper() == "MOVE_BASE":
                source = action.parameters[1].value.lower()
                destination = action.parameters[2].value.lower()
                marker = self._utils.get_arc_marker(source, destination)
                marker.color.r = move_base_config["color"]["r"]
                marker.color.g = move_base_config["color"]["g"]
                marker.color.b = move_base_config["color"]["b"]
                marker.color.a = move_base_config["color"]["a"]
                move_base_markers.append(marker)
            elif action.name.upper() == "PICK":
                for marker in kb_markers[0]:
                    if marker.text.upper() == action.parameters[2].value:
                        marker.mesh_use_embedded_materials = True
                        marker.color.r = pick_config["color"]["r"]
                        marker.color.g = pick_config["color"]["g"]
                        marker.color.b = pick_config["color"]["b"]
                        marker.color.a = pick_config["color"]["a"]
                        marker.scale.x = marker.scale.y = marker.scale.z = pick_config[
                            "scale"
                        ]
                        break

        kb_markers.append(move_base_markers)

    def visualise(self, kb_markers, kb_data=None):
        """
        - Modify markers based on pick action.
        - Add markers for move_base actions

        :param kb_markers: markers from :class:`mir_planning_visualisation.kb_visualiser.KnowledgeBaseVisualiser`
        :type kb_markers: list (lists (visualization_msgs.msg.Marker))
        :param kb_data: data from knowledge base
        :type kb_data: dict
        :return: all markers to be published
        :rtype: list (visualization_msgs.msg.Marker)

        """
        if kb_data is None:
            return None

        self._utils.marker_counter = 10000  # to avoid marker id repetition
        if self._complete_plan:
            self._set_markers_for_complete_plan(kb_markers, kb_data)

        # flatten the list of markers
        return sum(kb_markers, [])
