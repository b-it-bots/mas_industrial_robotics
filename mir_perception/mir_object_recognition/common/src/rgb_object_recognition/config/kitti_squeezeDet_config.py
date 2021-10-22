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
    mc                         = base_model_config('KITTI')

    mc.IMAGE_WIDTH             = 640
    mc.IMAGE_HEIGHT            = 480
    mc.BATCH_SIZE              = 8

    mc.WEIGHT_DECAY            = 0.0001
    mc.LEARNING_RATE           = 0.01
    mc.DECAY_STEPS             = 10000
    mc.MAX_GRAD_NORM           = 1.0
    mc.MOMENTUM                = 0.9
    mc.LR_DECAY_FACTOR         = 0.5

    mc.LOSS_COEF_BBOX          = 5.0
    mc.LOSS_COEF_CONF_POS      = 75.0
    mc.LOSS_COEF_CONF_NEG      = 100.0
    mc.LOSS_COEF_CLASS         = 1.0
    
    # probability of output to keep
    mc.PLOT_PROB_THRESH        = 0.4
    mc.NMS_THRESH              = 0.4
    mc.PROB_THRESH             = 0.005
    mc.TOP_N_DETECTION         = 64

    mc.DATA_AUGMENTATION       = False
    mc.DRIFT_X                 = 150
    mc.DRIFT_Y                 = 100
    mc.EXCLUDE_HARD_EXAMPLES   = False

    mc.ANCHOR_BOX              = set_anchors(mc)
    mc.ANCHORS                 = len(mc.ANCHOR_BOX)
    mc.ANCHOR_PER_GRID         = 9

    return mc

def set_anchors(mc):
    H, W, B = 30, 40, 9

    #detection with 9 anchors
    #update: 21.06.2021

    # used in bmt
    #anchor_shapes = np.reshape(
    #  [np.array(
    #      [[92.02941071, 115.09681643], [200.65465954,229.89014957],
    #       [167.39743532, 113.50341595], [259.76506943, 119.82479679],
    #       [136.6747144, 170.01330096], [101.93293109, 251.32520022],
    #       [53.74847805, 51.47580068], [263.39513927, 336.70239122],
    #       [344.11060212, 199.75904517]])] * H * W,
    #  (H, W, B, 2)
    #)

    anchor_shapes = np.reshape(
      [np.array(
             [[ 88.99409368, 42.66395702],
             [192.3702808,  178.53317949],
             [ 41.63787236,  38.45010246],
             [ 98.34916593,  95.40313721],
             [315.56379179, 244.77646964],
             [157.59156338,  93.99741645],
             [ 91.63670505, 170.70665342],
             [223.24388109, 305.69230947],
             [ 60.49908951,  76.76089189]]
      )] * H * W,
      (H, W, B, 2)
    )

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
