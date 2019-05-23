# Author: Bichen Wu (bichen@berkeley.edu) 08/25/2016

"""Model configuration for pascal dataset"""
import os,sys

BASE_DIR = os.path.dirname(os.path.abspath('__file__'))
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, 'src/config/'))

import numpy as np

from config_required import base_model_config

def kitti_squeezeDet_config():
  """Specify the parameters to tune below."""
  mc                       = base_model_config('KITTI')

  mc.IMAGE_WIDTH           = 640
  mc.IMAGE_HEIGHT          = 480
  mc.BATCH_SIZE            = 8

  mc.WEIGHT_DECAY          = 0.0001
  mc.LEARNING_RATE         = 0.01
  mc.DECAY_STEPS           = 10000
  mc.MAX_GRAD_NORM         = 1.0
  mc.MOMENTUM              = 0.9
  mc.LR_DECAY_FACTOR       = 0.5

  mc.LOSS_COEF_BBOX        = 5.0
  mc.LOSS_COEF_CONF_POS    = 75.0
  mc.LOSS_COEF_CONF_NEG    = 100.0
  mc.LOSS_COEF_CLASS       = 1.0

  mc.PLOT_PROB_THRESH      = 0.4
  mc.NMS_THRESH            = 0.4
  mc.PROB_THRESH           = 0.005
  mc.TOP_N_DETECTION       = 64

  mc.DATA_AUGMENTATION     = True
  mc.DRIFT_X               = 150
  mc.DRIFT_Y               = 100
  mc.EXCLUDE_HARD_EXAMPLES = False

  mc.ANCHOR_BOX            = set_anchors(mc)
  mc.ANCHORS               = len(mc.ANCHOR_BOX)
  #mc.ANCHOR_PER_GRID       = 9
  mc.ANCHOR_PER_GRID       = 7

  return mc

def set_anchors(mc):
  H, W, B = 30, 40, 7
  anchor_shapes = np.reshape(
      [np.array(
          [[  155.,  188.], [ 101., 129.], [ 273.,  127.],
           [ 104.,  254.], [  252.,  281.], [ 168., 113.],
           [ 57., 58.]])] * H * W,
      (H, W, B, 2)
  )
  # H, W, B = 30, 40, 9
  # anchor_shapes = np.reshape(
  #     [np.array(
  #         [[  197.,  237.], [ 77., 111.], [ 156.,  140.],
  #         [ 262.,  129.], [  48.,  51.], [ 99., 218.],
  #         [ 325., 228.], [  135., 76.], [  256.,  345.]])] * H * W,
  #     (H, W, B, 2)
  # )
  center_x = np.reshape(
      np.transpose(
          np.reshape(
              np.array([np.arange(1, W+1)*float(mc.IMAGE_WIDTH)/(W+1)]*H*B), 
              (B, H, W)
          ),
          (1, 2, 0)
      ),
      (H, W, B, 1)
  )
  center_y = np.reshape(
      np.transpose(
          np.reshape(
              np.array([np.arange(1, H+1)*float(mc.IMAGE_HEIGHT)/(H+1)]*W*B),
              (B, W, H)
          ),
          (2, 1, 0)
      ),
      (H, W, B, 1)
  )
  anchors = np.reshape(
      np.concatenate((center_x, center_y, anchor_shapes), axis=3),
      (-1, 4)
  )

  return anchors
