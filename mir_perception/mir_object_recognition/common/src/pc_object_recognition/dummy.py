

import torch
from models.dgcnn import DGCNN
import torch.nn as nn

cuda = False
args = {
    "emb_dims": 1024,
    "dropout": 0.5,
    "k": 20
}

device = torch.device("cuda" if cuda else "cpu")
model = DGCNN(args).to(device)
model = nn.DataParallel(model)
model.load_state_dict(torch.load('mir_perception_models/mir_pointcloud_object_recognition_models/common/models/dgcnn/all/model.t7'))