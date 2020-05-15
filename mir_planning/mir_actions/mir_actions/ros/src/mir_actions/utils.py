#! /usr/bin/env python

from __future__ import print_function

class Utils(object):

    """
    Utility functions for :ref:`mir_actions` packages
    """

    @staticmethod
    def get_value_of(params, desired_key):
        """Get the value of ``desired_key`` from ``params``
        If none of the key matches, returns None

        :param params: all key value pairs
        :type params: list (diagnostic_msgs/KeyValue)
        :param desired_key: query key
        :type desired_key: str
        :return: value corresponding ``desired_key``
        :rtype: str | None

        """
        for param in params:
            if desired_key == param.key:
                return param.value
        return None
