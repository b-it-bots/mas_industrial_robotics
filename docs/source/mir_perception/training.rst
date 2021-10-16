.. _training:

Training models
===============

.. _training_pointcloud_classification:

Traning point cloud classification
----------------------------------

Collect point cloud dataset as describe in :ref:`3d_dataset_collection`, and 
preprocess them as explained in :ref:`3d_dataset_preprocessing`.

To train the models, you need to clone `this repo <https://github.com/mhwasil/pointcloud_classification>`_, and 
install the requirements to train the model.

Training deep learning model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  The supported models are available under `config/config.yaml`.

  .. code-block:: bash
    
    #python trainer.py --model <model_name> --train
    #an example for training DGCNN with color (DGCNNC)

    python trainer.py --model DGCNNC --train


Adding new model
^^^^^^^^^^^^^^^^

ToDo


.. _training_2d_object_detection:

Training 2D object detection
----------------------------

ToDo

.. note::
  
  You can train the model on `H-BRS Scientific Computing Cluster <https://wr0.wr.inf.h-brs.de/wr/index.html>`_,
  provided that you have access to it. The tutorial on how to submit job and train 
  your model on the cluster can be found `here <https://github.com/mhwasil/pointcloud_classification/blob/master/hbrs_cluster_usage.md>`_.