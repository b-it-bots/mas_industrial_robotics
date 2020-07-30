.. _contributing_ros:

Robot Operating System (ROS)
=============================

ROS packages naming and structure
----------------------------------

Creating a new package
^^^^^^^^^^^^^^^^^^^^^^^

* Naming

  In order to create a new ROS package for one of the repositories some rules need to be considered:

  1. The package name has always the following format:

    .. code-block:: bash

      prefix_my_package_name

  2. Use the right prefix for every repository:

    a. mas_domestic_robotics: *mdr_*
    b. mas_industrial_robotics: *mir_*
    c. mas_common_robotics: *mcr_*

  3. Use lowercase.
  4. Separate words in the package name by underscores (_).

  Examples for creating packages according to the above described rules are as follows:

  .. code-block:: bash

    catkin create_pkg mdr_grasp_planning
    catkin create_pkg mir_whole_body_control
    catkin create_pkg mcr_object_detection

* Folder structure

  Every ROS package within our repositories has to strictly match the following structure:

  .. code-block:: bash

    .
    ├── common
    │   ├── config
    │   ├── include
    │   │   └── <package_name>
    │   ├── src
    │   │   └── <package_name>
    │   ├── test
    │   └── tools
    ├── ros
    │   ├── config
    │   ├── include
    │   │   └── <package_name>
    │   ├── launch
    │   │   └── <package_name>.launch
    │   ├── rviz
    │   │   └── <package_name>.rviz
    │   ├── scripts
    │   │   └── <package_name>
    │   ├── src
    │   │   └── <package_name>_node
    │   ├── test
    │   └── tools
    ├── CMakeLists.txt
    ├── package.xml
    ├── setup.py
    └── README.md

  In short:
    * ROS-independent code goes into the `common` folder
    * the `ros` folder contains a ROS-wrapper for the functionality you are adding

Meta-packages
^^^^^^^^^^^^^^

If the package you are creating is meant to contain other packages inside of it, it needs to have instead the following structure:

.. code-block:: bash

  ./<meta_package_name>
  └── <meta_package_name>
      ├── CMakeLists.txt
      ├── package.xml
      └── README.md

.. note::

  It is **extremely** important to maintain your *package.xml* up to date with its dependencies.
  Not doing so results in the need of specialized tools or manual inspection of launch files and
  source code to discover your package dependencies.

Messages, services and actions
-------------------------------

Creating a new message, service or action.
If your package defines its own messages, services or actions you should add them to the corresponding meta-package:

.. code-block:: bash

  ./<package_name>_msgs
  ├── action
  │   ├── MyAction.action
  ├── msg
  │   ├── MyMessage.msg
  ├── srv
  │   └── MyService.srv
  ├── CMakeLists.txt
  ├── package.xml
  └── README.md

.. note::

  The *srv* file name should start with verb i.e. *RecognizeImage.srv*

Depending on the repository you are working on, the meta-package is related to the domain, e.g. *mdr_planning_msgs* or *mdr_navigation_actions*

Linting
--------

Running **roslint** with catkin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before merging into the main repository *roslint* is ran on all merge requests.
Unless all errors are resolved the merge request will be rejected. To test if your changes would pass the *roslint* test locally:

* Add the following lines to your `CMakelists.txt`:

  .. code-block:: bash

    find_package(catkin REQUIRED COMPONENTS roslint ...)

    roslint_python()  # pep8 linting
    roslint_cpp()     # ROS wrapper of Google's cpplint

  Your *package.xm* should include *roslint* as a build dependency:

  .. code-block:: bash

    <build_depend>roslint</build_depend>

* Build target roslint:

  * with `catkin_make`:

    .. code-block:: bash

      catkin_make roslint_<package_name>

  * with `catkin_tools`:

    .. code-block:: bash

      catkin build --no-deps <package_name> --make-args roslint_<package_name>

* If build fail copy and execute the gray line that looks something like the following to see more detailed errors:

  .. code-block:: bash

    cd <package_source_directory>
    catkin build --get-env <package_name> | catkin env -si  /usr/bin/make roslint --jobserver-fds=6,7 -j; cd -


Running **catkin_lint**
^^^^^^^^^^^^^^^^^^^^^^^^^

You should also make sure that the *catkin_lint* tests pass;
running it from the root of your catkin workspace you can run:

.. code-block:: bash

  catkin_lint --strict --ignore CRITICAL_VAR_APPEND,LINK_DIRECTORY src/mas_domestic_robotics


See Also:

* `roslint <http://wiki.ros.org/roslint>`_
* `catkin_lint <http://fkie.github.io/catkin_lint/>`_

Proposed linters:

* `C++ <http://clang.llvm.org/extra/clang-tidy/>`_
* `Python <https://pypi.python.org/pypi/pep8>`_
* `ROS <http://wiki.ros.org/roslint>`_
