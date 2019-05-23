import os, sys

BASE_DIR = os.path.dirname(os.path.abspath('__file__'))
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, 'src/dataset/'))

from kitti import kitti
from pascal_voc import pascal_voc
