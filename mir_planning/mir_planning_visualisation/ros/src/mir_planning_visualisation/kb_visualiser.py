from __future__ import print_function

import yaml
import rospy

from visualization_msgs.msg import MarkerArray, Marker
from rosplan_knowledge_msgs.srv import GetAttributeService
from rosplan_knowledge_msgs.msg import KnowledgeItem

from mir_planning_visualisation.utils import Utils

class KnowledgeBaseVisualiser(object):

    """
    Visualise propositions from knowledge base
    Currently supported attributes are `on`, `in`, `at` and `stored`.
    """

    def __init__(self):
        # read ros params
        self._debug = rospy.get_param('~debug', False)
        self._kb_server_facts_topic = '~kb_server_facts_topic'
        self._kb_server_goals_topic = '~kb_server_goals_topic'
        marker_config_file = rospy.get_param('~plan_marker_color_config', None)

        plan_marker_config = None
        with open(marker_config_file) as file_obj:
            plan_marker_config = yaml.safe_load(file_obj)
        if plan_marker_config is None:
            raise Exception('Plan marker config not provided.')
        self._goal_obj_config = plan_marker_config['goal']

        # class variables
        self._utils = Utils()
        self._prev_data = self._get_visualisable_data_from_facts_and_goals()
        self._prev_markers = None

    def visualise(self):
        """
        - Query KB for all the facts.
        - Create data for facts with specific attribute.
        - Create markers from that data.
        - Return markers and data

        :returns: (list of visualization_msgs.Marker, dict)

        """
        facts = self._get_response_from_kb(self._kb_server_facts_topic)
        if facts is None or (isinstance(facts, list) and len(facts) == 0):
            rospy.loginfo('No valid response')
            return None, None

        goals = self._get_response_from_kb(self._kb_server_goals_topic)
        if goals is None or (isinstance(goals, list) and len(goals) == 0):
            goals = []

        data = self._get_visualisable_data_from_facts_and_goals(facts, goals)
        if data != self._prev_data:
            self._utils.marker_counter = 0
            self._prev_data = data
            self._prev_markers = self._get_markers_from_data(data)

        return self._prev_markers, self._prev_data

    def _get_response_from_kb(self, server_topic):
        if self._debug:
            rospy.loginfo('Waiting for service: ' + server_topic)
        rospy.wait_for_service(server_topic)
        try:
            get_response = rospy.ServiceProxy(server_topic, GetAttributeService)
            response = get_response()
            if len(response.attributes) == 0:
                rospy.logwarn('No propositions in KB')
                return None
            else:
                if self._debug:
                    rospy.loginfo('Found propositions: ' + str(len(response.attributes)))
                facts = [attribute for attribute in response.attributes 
                         if attribute.knowledge_type == KnowledgeItem.FACT]
                resp = [(fact.attribute_name, {kv.key:kv.value for kv in fact.values})
                        for fact in facts]
                return resp
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed: %s'%e)
            return None

    def _get_markers_from_data(self, data):
        markers = []

        obj_on_ws = []
        for ws_name, obj_list in data['obj_on_ws'].iteritems():
            obj_markers = self._utils.get_markers_from_obj_on_ws(obj_list,
                                                                 ws_name,
                                                                 data['obj_in_container'])
            obj_on_ws.extend(obj_markers)
        markers.append(obj_on_ws)

        obj_on_robot_markers = []
        for platform, obj in data['obj_on_robot'].iteritems():
            obj_marker = self._utils.get_markers_from_obj_on_robot(obj,
                                                                   platform,
                                                                   data['robot_ws'])
            if obj_marker:
                obj_on_robot_markers.append(obj_marker)
        markers.append(obj_on_robot_markers)

        goal_obj_on_ws = []
        for ws_name, obj_list in data['goal_obj_on_ws'].iteritems():
            obj_markers = self._utils.get_markers_from_obj_on_ws(obj_list,
                                                                 ws_name,
                                                                 data['goal_obj_in_container'],
                                                                 is_goal=True)
            for marker in obj_markers:
                marker.text += "GOAL"
                marker.color.r = self._goal_obj_config['color']['r']
                marker.color.g = self._goal_obj_config['color']['g']
                marker.color.b = self._goal_obj_config['color']['b']
                marker.color.a = self._goal_obj_config['color']['a']
                marker.scale.x = marker.scale.y = marker.scale.z = self._goal_obj_config['scale']
            goal_obj_on_ws.extend(obj_markers)
        markers.append(goal_obj_on_ws)

        markers.append(self._utils.get_markers_for_youbot(data['robot_ws']))
        markers.append(self._utils.get_markers_from_ws_pos())
        return markers

    def _get_visualisable_data_from_facts_and_goals(self, facts=[], goals=[]):
        obj_on_ws = {}
        robot_ws = 'start'
        obj_on_robot = {}
        obj_in_container = {}
        goal_obj_on_ws = {}
        goal_obj_in_container = {}

        for fact in facts:
            if self._debug:
                print(fact)

            if fact[0] == 'on':
                if fact[1]['l'] not in obj_on_ws:
                    obj_on_ws[fact[1]['l']] = []
                if 'o' in fact[1]:
                    obj_on_ws[fact[1]['l']].append(fact[1]['o'])

            if fact[0] == 'stored':
                obj_on_robot[fact[1]['rp']] = fact[1]['o']

            if fact[0] == 'at':
                robot_ws = fact[1]['l']

            if fact[0] == 'in':
                container = fact[1]['hole']
                if container not in obj_in_container:
                    obj_in_container[container] = []
                if 'peg' in fact[1]:
                    obj_in_container[container].append(fact[1]['peg'])

        for fact in goals:
            if self._debug:
                print(fact)

            if fact[0] == 'on':
                if fact[1]['l'] not in goal_obj_on_ws:
                    goal_obj_on_ws[fact[1]['l']] = []
                if 'o' in fact[1]:
                    goal_obj_on_ws[fact[1]['l']].append(fact[1]['o'])

            if fact[0] == 'in':
                container = fact[1]['hole']
                if container not in goal_obj_in_container:
                    goal_obj_in_container[container] = []
                if 'peg' in fact[1]:
                    goal_obj_in_container[container].append(fact[1]['peg'])

        data = {}
        data['obj_on_ws'] = obj_on_ws
        data['robot_ws'] = robot_ws
        data['obj_on_robot'] = obj_on_robot
        data['obj_in_container'] = obj_in_container
        data['goal_obj_on_ws'] = goal_obj_on_ws
        data['goal_obj_in_container'] = goal_obj_in_container
        return data
