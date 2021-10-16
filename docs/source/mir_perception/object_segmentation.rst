.. _object_segmentation:

Object Segmentation
===================

.. _3d_object_segmentation:

3D object segmentation
-------------------------

Our scene segmentation works with the assumption that the table has to be parallel 
to the ground as we use 
`SampleConsensusModelPerpendicularPlane <https://pointclouds.org/documentation/classpcl_1_1_sample_consensus_model_perpendicular_plane.html>`_
from PCL library. We keep points which are perpendicular to `z` axis.

There are two packages you can launch in order to segment table top point clouds:

*mir_object_segmentation* package 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`mir_object_segmentation <https://github.com/b-it-bots/mas_industrial_robotics/tree/melodic/mir_perception/mir_object_segmentation>`_ 
package can be used to accumulate clouds from different views (if necessary),
register them, and then segment the objects above the table.

**Usage**

* Find plane height

  To find the object height, you have to send the following messages to `event_in`
  topic:

  .. code-block:: bash

    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_start
    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_add_cloud_start
    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_find_plane

  The plane height will be published to `workspace_height` topic:

  .. code-block:: bash
    
    /mir_perception/scene_segmentation/output/workspace_height


* Segment objects from one view point

  .. code-block:: bash

    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_start
    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_add_cloud_start
    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_segment
    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_stop

* Segment objects from one view point

  You can segment table top objects from multiple viewpoints. You can do this by 
  manually moving the camera position to a new view point, and then register 
  the cloud with the existing ones.

  .. code-block:: bash

    # accumulate point cloud from the 1st view point
    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_start
    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_add_cloud_start
    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_add_cloud_stop

    # accumulate point cloud from the 2nd view point
    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_add_cloud_start
    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_add_cloud_stop

    # segment accumulated cloud and stop segmentation
    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_segment
    rostopic pub /mir_perception/scene_segmentation/event_in std_msgs/String e_stop


.. hint:: Published msgs from mir_object_segmentation

  * Object list
    
    .. code-block:: bash

      /mir_perception/scene_segmentation/output/object_list

  * Bounding Boxes (for visualization in Rviz)

    .. code-block:: bash

      /mir_perception/scene_segmentation/output/bounding_boxes
    
  * Object labels (for visualization in Rviz)

    .. code-block:: bash
      
      /mir_perception/scene_segmentation/output/labels

  * Debug pointcloud (shows filtered input to plane segmentation)

    .. code-block:: bash

      /mir_perception/scene_segmentation/output/debug_cloud
      
  * Object clusters: segmented point clouds

    .. code-block:: bash

      /mir_perception/scene_segmentation/output/tabletop_clusters

  * Workspace height:

    .. code-block:: bash
      
      /mir_perception/scene_segmentation/output/workspace_height


*mir_object_recognition* package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

`mir_object_recognition` uses the same `scene_segmentation` method 
as in `mir_object_segmentation`, but it does not accumulate point clouds. It only 
takes one view and then segment table top objects.

**Usage**

* Segment objects

  .. code-block:: bash

    rostopic pub /mir_perception/multimodal_object_recognition/event_in std_msgs/String e_start

**Published messages**

  .. code-block:: bash

    /mcr_perception/object_detector/object_list
    /mir_perception/multimodal_object_recognition/output/bounding_boxes
    /mir_perception/multimodal_object_recognition/output/debug_cloud_plane
    /mir_perception/multimodal_object_recognition/output/pc_labels
    /mir_perception/multimodal_object_recognition/output/pc_object_pose_array
    /mir_perception/multimodal_object_recognition/output/rgb_labels
    /mir_perception/multimodal_object_recognition/output/rgb_object_pose_array
    /mir_perception/multimodal_object_recognition/output/tabletop_cluster_pc
    /mir_perception/multimodal_object_recognition/output/tabletop_cluster_rgb
    /mir_perception/multimodal_object_recognition/output/workspace_height
