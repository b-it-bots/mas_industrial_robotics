from rosplan_dispatch_msgs.msg import ActionDispatch
from diagnostic_msgs.msg import KeyValue

class Utils(object):

    @staticmethod
    def get_action_msg_from_string(string, action_id):
        """Create and return Action obj from string of type '(word word word ...)'

        :string: string
        :action_id: int
        :returns: rosplan_dispatch_msgs/ActionDispatch

        """
        # remove parenthesis, convert str to upper case and split at spaces
        action_param_list = string[1:-1].upper().split()
        name = action_param_list[0]
        params = [KeyValue('param_'+str(i+1), param) for i, param in enumerate(action_param_list[2:])]
        params.insert(0, KeyValue('robot_name', action_param_list[1]))
        return ActionDispatch(action_id=action_id, name=name, parameters=params)
