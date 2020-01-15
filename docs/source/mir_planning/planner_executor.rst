.. _mir_planner_executor:

Planner executor
================

``planner_executor`` is the main executable which is executed when ``planner_executor.launch`` is launched.
It creates a bunch of ``BaseExecutorAction`` objects.
When a execute plan goal is received, it call individual action's ``execute``
function.

Most of the time, this ``execute`` function will change the names of the
parameters obtained from planner to something that makes sense. After that, it
will call ``run`` function which 

  - creates a goal of the action server
  - sends this goal to action server
  - wait for the server to respond within certain time duration
  - if the server responds with success, it will return ``true`` otherwise ``false``

Some actions are ``Combined``, which means, it has most of the things in common
at planning and execution level but has different server (for example,
*perceive location* and *perceive cavity*). This will have a check on one of
the parameters. Based on this check either one of the server will be called as
described above.


Usage
-----

Normal use
^^^^^^^^^^

.. code-block:: bash

    roslaunch mir_planner_executor planner_executor.launch


Testing
^^^^^^^

.. code-block:: bash

    roslaunch mir_planner_executor planner_executor.launch
    roslaunch mir_planner_executor planner_executor_test.launch


OOP Structure
-------------

.. code-block:: html

    BaseExecutorAction
        |
        '- ExecutorAction
        |      |
        |      '- MoveAction
        |      |
        |      '- PickAction
        |      |
        |      '- PlaceAction
        |      |
        |      '- StageAction
        |      |
        |      '- UnstageAction
        |      |
        |      '- BasePerceiveAction
        |      |      |
        |      |      '- PerceiveAction
        |      |      |
        |      |      '- PerceiveCavityAction
        |      |
        |      '- BaseInsertAction
        |             |
        |             '- InsertAction
        |             |
        |             '- InsertCavityAction
        |
        '- CombinedPerceiveAction
        |
        '- CombinedInsertAction
