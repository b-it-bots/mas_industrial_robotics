.. _mir_knowledge_base_analyzer:

Knowledge base analyzer
=======================

#. query the knowledge base and tells you if there is pending goals
#. query the knowledge base and informs if there is new knowledge

Usage
-----

Get objects for a particular location::

    rostopic pub /mir_knowledge_base_analyzer/knowledge_base_queries/query_param std_msgs/String "WS02"
    rostopic pub /mir_knowledge_base_analyzer/knowledge_base_queries/query std_msgs/String "get_objects_at_location"
    rostopic echo /mir_knowledge_base_analyzer/knowledge_base_queries/objects_at_location


Get current location of robot::

    rostopic pub /mir_knowledge_base_analyzer/knowledge_base_queries/query std_msgs/String "get_robot_location"
    rostopic echo /mir_knowledge_base_analyzer/knowledge_base_queries/robot_location
