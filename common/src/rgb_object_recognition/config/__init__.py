import sys
import os 

BASE_DIR = os.path.dirname(os.path.abspath('__file__'))
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, 'src/config'))
#print(sys.path)

from kitti_model_config import kitti_model_config
#from kitti_vgg16_config import kitti_vgg16_config
#from kitti_res50_config import kitti_res50_config
from kitti_squeezeDet_config import kitti_squeezeDet_config
#from kitti_squeezeDetPlus_config import kitti_squeezeDetPlus_config