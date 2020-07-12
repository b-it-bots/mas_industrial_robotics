.. _dataset:

Dataset
========

.. _dataset_collection:

Dataset collection
----------------------

.. _3d_dataset_collection:

3D dataset collection
^^^^^^^^^^^^^^^^^^^^^^

We use a rotating table to collect point cloud data.

.. note::

  This only works with a single object.

Setup:

  1. Using robot arm camera

  2. Using camera

    * Launch the camera

      Go to :ref:`realsense2_camera` for more information about the camera.

    * Apply static transform from camera_frame to base_link as explained in :ref:`realsense2_camera`

      Make sure the pointcloud of the plane is parallel to the gorund on rviz by transforming/rotating it.

      .. note::

        Passthrough filter will not work if it's not parallel to the ground

    * Then launch multimodal object recognition

      .. code-block:: bash

        roslaunch mir_object_recognition multimodal_object_recognition.launch debug_mode:=true

      .. note::

        To enable dataset collection, it requires to be in *debug_mode*. You can also
        point to a specifi logdir to save the data e.g. logdir:=/home/robocup/cloud_dataset

    * Trigger data collection mode

      .. code-block:: bash

        rostopic pub /mir_perception/multimodal_object_recognition/event_in std_msgs/String e_data_collection

    * Start collectiong dataset

      .. code-block:: bash

        rostopic pub /mir_perception/multimodal_object_recognition/event_in std_msgs/String e_start

    * Stop data collection mode

      .. code-block:: bash

        rostopic pub /mir_perception/multimodal_object_recognition/event_in std_msgs/String e_stop_data_collection

.. _2d_dataset_collection:

2D dataset collection
^^^^^^^^^^^^^^^^^^^^^^

* Collect dataset using rotating table
* Collect dataset using random surface
* Collect dataset during competition

.. _dataset_preprocessing:

Dataset preprocessing
-----------------------

.. _2d_dataset_preprocessing:

2D dataset preprocessing
^^^^^^^^^^^^^^^^^^^^^^^^^^

* Object detection

  * Download labelme
  * Label the data
  * Augment data
  * Convert to a particular format: VOC, KITTI etc

