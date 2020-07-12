.. _mir_perceive_aruco:

Perceive mock
=============

Perceive an aruco cube on a workstation

Related to: ``mir_perceive_aruco_cube``

Goal parameter description
--------------------------

- ``location``: name of known/mapped location (e.g. ``WS01``, ``SH02``, ``PP01``)

Usage
-----

Make sure :ref:`mir_planning_bringup`'s launch file is already launched.

  .. code-block:: bash

    roslaunch mir_perceive_mock perceive_aruco_server.launch
    rosrun mir_perceive_mock perceive_aruco_client_test
