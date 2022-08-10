.. _docker:

Docker
######

The latest versions of docker-engine and docker-compose have to be installed 
before getting started. Please have a look at `docker's official website <https://docs.docker.com/get-started/overview/>`_ 
for more insights into the working and usage of docker images and docker containers.

It is highly recommended that you use docker containers to build and run your 
nodes rather than directly installing ROS and working with the MAS industrial software on your PC.

.. _using_mir_docker_images:

Using Our Images
================

Our docker images are available `here <https://github.com/orgs/b-it-bots/packages>`_.
The images provide a proper development environment -with ros pre-installed and without 
any missing dependencies- for the MAS industrial software. Additionally, the images 
also contain src packages which are prebuilt.

Directory structure in the image:

  .. code-block:: bash

    /home/robocup/<ros_distro>/catkin_ws
      ├── build
      ├── devel
      ├── logs
      ├── src
        ├── mas_industrial_robotics
        ├── mas_perception_msgs
        ├── mas_navigation
        ├── ...

Where `ros_distro` corresponds to the ros distribution i.e. `kinetic, melodic`.

The main github branch of the each ros distro should always reflect `latest` 
tag in the github registry, for example `kinetic` branch reflects 
`mas_industrial_robotics/industrial-kinetic:latest`, while the `devel` branch
always reflects `devel` tag, for example `kinetic` branch reflects  `mas_industrial_robotics/industrial-kinetic:devel`

* **Start the container**

  To start the container, `docker-compose <https://docs.docker.com/compose/install/>`_ 
  is required, with which it is easier to define the environment and volumes from 
  the host PC to the container. 

  .. code-block:: bash
    
    docker-compose -f start-container.yaml up industrial_<ros_distro>

  If there is no local image, docker will automatically pull from MIR github 
  container registry. 

  The images can also be pulled manually without docker-compose.

  .. code-block:: bash

    #kinetic image
    docker pull  ghcr.io/b-it-bots/mas_industrial_robotics/industrial-kinetic:latest

    #melodic image
    docker pull ghcr.io/b-it-bots/mas_industrial_robotics/industrial-melodic:latest

* **Log in to the container**

  .. code-block:: bash
    
    docker exec -it mas_industrial_robotics_industrial_kinetic_1 /bin/bash

.. tip::

  The container is set up to use `host` network, and therefore when you run 
  roscore or ros packages in either host or container, and both will have access 
  to ros communication infrastructure such as topics and services.


.. _mounting_volumes:

Mounting Local Volumes
======================

Since the codes in the image are not persistent, which means they will be 
removed when the images are rebuilt, it is recommended to mount your local disk 
containing the repository to the container.

To mount your local disk to the container, you need to modify `start-container.yaml` 
under `mas_industrial_robotics` repository. The following example shows how you 
can mount your `rviz` configuration and `mas_industrial_robotics` repository which 
is mounted to `/home/robocup/<ros_distro>/catkin_ws/src/external` in the container.

  .. code-block:: yaml

    # under volume add your local disk
    volumes:
      - $HOME/.rviz:/home/robocup/.rviz
      - $HOME/catkin_ws/mas_industrial_robotics:/home/robocup/<ros_distro>/catkin_ws/src/external/mas_industrial_robotics

Since there are now two `mas_industrial_robotics` repositories, you have to do 
the followings to make the build successful:

  * Remove or `CATKIN_IGNORE` the repository that comes with the image under `src/mas_industrial_robotics`.
  * `catkin clean` to remove the prebuilt packages from the image
  * Rebuild the package e.g. `catkin build mir_object_recognition`

Now, you can make changes locally in your PC using your favourite IDE, and build 
the package inside the container.

.. note::

  This was only tested with Linux.


.. _creating_own_image:

Creating Your Own Image
=======================

You can create your own image by using b-it-bots image as base:

.. code-block:: bash

  FROM ghcr.io/b-it-bots/mas_industrial_robotics/industrial-melodic:latest

  USER robocup

  # add your modification e.g. install tensorflow
  RUN pip install tensorflow
