.. _dataset:

Dataset
========

.. _dataset_collection:

Dataset collection
----------------------

.. _3d_dataset_collection:

3D dataset collection
^^^^^^^^^^^^^^^^^^^^^^

Objects are placed on a rotating table such that it can capture the objects from 
different angle. However, this can be done manually on a normal table and change 
the object orientation manually.

.. note::

  This only works with a single object.

Setup:

  1. Using external camera

    * Launch the camera

      Go to :ref:`realsense2_camera` for more information about the camera.

    * Apply static transform from camera_frame to base_link as explained in :ref:`realsense2_camera`

      Make sure the pointcloud of the plane is parallel to the gorund on rviz by transforming/rotating it.

      .. note::

        Passthrough filter will not work if it's not parallel to the ground

    * Launch multimodal object recognition

      .. code-block:: bash

        roslaunch mir_object_recognition multimodal_object_recognition.launch debug_mode:=true

      .. note::

        To enable dataset collection, it requires to be in *debug_mode*. You can also
        point to a specifi logdir to save the data e.g. logdir:=/home/robocup/cloud_dataset.

    * Start collectiong dataset

      .. code-block:: bash

        rostopic pub /mir_perception/multimodal_object_recognition/event_in std_msgs/String e_start


  2. Using robot arm camera

    * Bringup the robot

    * Start `multimodal_object_recognition` and continue with the next steps as described previously.
    
 .. note::

    The segemented point clouds are saved in the `logdir`.

.. _2d_dataset_collection:

2D dataset collection
^^^^^^^^^^^^^^^^^^^^^^

Images can be collected using the robot camera or external camera.
They can also be collected using `easy augment too <https://github.com/santoshreddy254/easy_augment>`_ 
which use Intel Realsense D435 camera to capture the image and automatically 
annotate them for 2D object detection.


.. _dataset_preprocessing:

Dataset preprocessing
-----------------------

Before training training the model, the data should be preprocessed, and this 
includes but not limited to *removing bad data*, *normalization*, and converting 
it to the required format such as *h5* for point clouds and *VOC* or *KITTI* for 
images.

.. _3d_dataset_preprocessing:

3D dataset preprocessing
^^^^^^^^^^^^^^^^^^^^^^^^

An example of the data directory structure:

.. code-block:: bash

  b-it-bots_atwork_dataset
  ├── train
  │   ├── AXIS
  |       ├── axis_0001.pcd
  |       ├── ...
  │   ├── ...
  ├── test
  │   ├── AXIS
  |       ├── axis_0001.pcd
  |       ├── ...
  │   ├── ...

The dataset preprocessing can be found in `this notebook 
<https://github.com/mhwasil/pointcloud_classification/blob/master/dataset/b-it-bots_dataset_preprocessing.ipynb>`_.

It will generate `pgz` files containing a dictionary of objects consisting of `x y z r g b` and label.


.. _2d_dataset_preprocessing:

2D dataset preprocessing
^^^^^^^^^^^^^^^^^^^^^^^^^^

* Create semantic labels using `labelme <https://github.com/wkentaro/labelme>`_.
* Convert the semantic labels using `labelme2voc <https://github.com/mhwasil/labelme/blob/master/examples/bbox_detection/labelme2voc.py>`_.
* If KITTI dataset is required, convert VOC dataset to KITTI using 
  `vod-converter <https://github.com/umautobots/vod-converter>`_
