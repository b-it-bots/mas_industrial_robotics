.. _object_recognition:

Object recognition
==================

Our object recognition comes from two different modalities, namely 3D based object recognition and 
2D object detection and recognition.

.. _3d_object_recognition_model:

3D object recognition models
----------------------------

`Our 3D object recognition node <https://github.com/b-it-bots/mas_industrial_robotics/blob/melodic/mir_perception/mir_object_recognition/ros/script/pc_object_recognizer_node>`_ 
uses segmented point clouds described in :ref:`3d_object_segmentation` as the input 
to the models. These segemented point clouds are published from 
`mir_object_recognition node <https://github.com/b-it-bots/mas_industrial_robotics/blob/melodic/mir_perception/mir_object_recognition/ros/src/multimodal_object_recognition_node.cpp>`_.

The tutorial for training the model is described in :ref:`training`.

We use two models for the 3D object recognition, namely:

* Random forest with Radial density distribution and 3D modified Fisher vector 
  (3DmFV) as features as described in `our paper <https://link.springer.com/chapter/10.1007/978-3-030-35699-6_48>`_.
* `Dynamic Graph CNN <https://github.com/WangYueFt/dgcnn>`_: an end-to-end point 
  cloud classification. However, in addition to points, we also incorporate colors as inputs.

You can change the classifier in the launch file

.. literalinclude:: ../../../mir_perception/mir_object_recognition/ros/launch/pc_object_recognition.launch
  :language: xml
  :lineno-start: 0
  :linenos:

Where:

* `model`: whether it is CNN based (`cnn_based`) or traditional ML estimators (`feature_based`) 
* `model_id`: the actual name of the model, available model ids: 

  * `cnn_based`: `dgcnn`
  * `feature_based`: `fvrdd`
* `dataset`: the dataset name where the model was trained on

.. _2d_object_recognition_model:

2D object recognition models
----------------------------

We use `squeezeDet <https://github.com/BichenWuUCB/squeezeDet>`_ for out 2D object detection model.
This is lightweight, one-shot object detection and classification.
The model can be changed in the `rgb_object_recognition.launch`

.. literalinclude:: ../../../mir_perception/mir_object_recognition/ros/launch/rgb_object_recognition.launch
  :language: xml
  :lineno-start: 0
  :linenos:

Where:

* `classifier`: the model used to detect and classify objects 
* `dataset`: the dataset used to train the model 

.. _mutlimodal_object_recognition:

Multimodal Object Recognition
-----------------------------

`multimodal_object_recognition_node <https://github.com/b-it-bots/mas_industrial_robotics/blob/melodic/mir_perception/mir_object_recognition/ros/src/multimodal_object_recognition_node.cpp>`_
coordinates the whole perception pipeline as described in the following items:

* Subscribes to rgb and point cloud topics
* Transforms point cloud to the target fram
* Finds 3D object clusters from the point cloud using `mir_object_segementation`
* Sends the 3D clusters to point cloud object recognizer (`pc_object_recognizer_node`)
* Sends the image to rgb object detection and recognition node (`rgb_object_recognized_node`)
* Waits until it gets results from both classifiers or if the timeout is reached
* Posts processing of the recognized objects

  * Applies filters for the objects
* Sends object_list to object_list_merger

**Trigger multimodal_object_recognition**

  .. code-block:: bash

    rostopic pub /mir_perception/multimodal_object_recognition/event_in std_msgs/String e_start

**Outputs**

  .. code-block:: bash

    /mcr_perception/object_detector/object_list
    /mir_perception/multimodal_object_recognition/output/workspace_height

**Visualization outputs**

  .. code-block:: bash

    /mir_perception/multimodal_object_recognition/output/bounding_boxes
    /mir_perception/multimodal_object_recognition/output/debug_cloud_plane
    /mir_perception/multimodal_object_recognition/output/pc_labels
    /mir_perception/multimodal_object_recognition/output/pc_object_pose_array
    /mir_perception/multimodal_object_recognition/output/rgb_labels
    /mir_perception/multimodal_object_recognition/output/rgb_object_pose_array
    /mir_perception/multimodal_object_recognition/output/tabletop_cluster_pc
    /mir_perception/multimodal_object_recognition/output/tabletop_cluster_rgb