import os, sys

BASE_DIR = os.path.dirname(os.path.abspath('__file__'))
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, 'src/nets/'))

from squeezeDet import SqueezeDet
#from squeezeDetPlus import SqueezeDetPlus
#from resnet50_convDet import ResNet50ConvDet
#from vgg16_convDet import VGG16ConvDet
