#! /usr/bin/env python

from __future__ import print_function

class Utils(object):

    """Utility functions for mir_yb_action packages"""

    @staticmethod
    def get_value_of(params, desired_key):
        """Get the value of `key` from params
        If none of the key matches, returns None

        :params: diagnostic_msgs/KeyValue []
        :desired_key: str
        :returns: str or None

        """
        for param in params:
            if desired_key == param.key:
                return param.value
        return None
