.. _mir_actions:

Actions
=======

A bunch of action servers for performing basic robocup actions.

The robocup @work domain is partitioned into 6 basic actions

- move base
- pick
- perceive
- place
- stage
- unstage

Each action is implemented as SMACH state machines which are wrapped with
``ActionServer``. An ``ActionClient`` needs to send request using
``GenericExecuteGoal``.

This request message contains a single dictionary kind of
message called ``parameters`` of type ``diagnostic_msgs/KeyValue[]``.

.. literalinclude:: ../../../mir_planning/mir_planning_msgs/ros/action/GenericExecute.action

Each server needs different information from this request message. Please see
the following files for detailed info:

.. toctree::
  :maxdepth: 1

  actions/move_base_safe.rst
  actions/perceive_location.rst
  actions/perceive_cavity.rst
  actions/perceive_mock.rst
  actions/pick_object.rst
  actions/place_object.rst
  actions/stage_object.rst
  actions/unstage_object.rst
  actions/insert_object.rst
  actions/insert_cavity.rst

Additionally, :ref:`mir_planner_executor` also sends ``next_action`` as one of the
parameter. This can be used by action servers to have parallel execution of arm
while the base is in motion to save som time. At the moment, only
:ref:`mir_move_base_safe` is using this information.

This package also contains ``Utils.py`` which contains utility functions for
action servers.

Usage
-----

.. code-block:: bash

   roscore
   roslaunch mir_actions run_action_servers.launch

To execute an action, call the corresponding action client with appropriate
arguments. For example, move base safe::

   rosrun mir_move_base_safe move_base_safe_client_test.py WS01
