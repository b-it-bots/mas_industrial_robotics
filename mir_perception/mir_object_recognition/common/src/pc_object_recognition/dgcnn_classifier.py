

from pc_object_recognition.cnn_based_classifiers import CNNBasedClassifiers
import torch
import torch.nn as nn

from pc_object_recognition.models.dgcnn import DGCNN
from pc_object_recognition.utils.data import infer_data
import torch.nn.functional as nnf
from torch.utils.data import DataLoader




class DGCNNClassifier(CNNBasedClassifiers):
    """
    Point cloud classifier using Dynamic graph CNN (DGCNN) 

    """

    def __init__(self, **kwargs):
        super(DGCNNClassifier, self).__init__(**kwargs)
        
        # extract argumen from key word arg

        self.num_classes = kwargs.get("num_classes", None)
        self.num_points = kwargs.get("num_points", None)
        self.cloud_dim = kwargs.get("cloud_dim", None)

        self.test_batch_size = 1 # only support batch size 1 for inferencing
        self.cuda = kwargs.get("cuda", False)
        self.model_path = kwargs.get("model_path", None)

        self.args = {
            "emb_dims": 1024,
            "dropout": 0.5,
            "k": 20
        }# contains the arguments that are passed for the DGCNN model

        # pytorch session

    def classify(self, pointcloud, center=True, rotate=True, pad=True): 
        """
        Classify point cloud

        :param pointcloud:    The input pointcloud (BxNxD), D can be XYZ or XYZRGB
        :type name:         numpy.array
        :param center:        If true, pointcloud will be centered
        :type name:         Bool
        :param rotate:        If true, pointcloud will be rotated to their principal axes
        :type name:         Bool
        :param pad:         If true, pointcloud will be padded
        :type name:         Bool

        :return:    Predicted label and probablity
        """
        
        test_loader = DataLoader(infer_data(num_points=self.num_points, pcl_path=pointcloud),
                                batch_size=self.test_batch_size, shuffle=True, drop_last=False)

        device = torch.device("cuda" if self.cuda else "cpu")
        model = DGCNN(self.args).to(device)
        model = nn.DataParallel(model)
        model.load_state_dict(torch.load(self.model_path))
        model = model.eval()

        for pcl, label in test_loader:

            print("inferencing ...")

            data = pcl.to(device)
    
            data = data.permute(0, 2, 1)

            logits = model(data)

            prob = nnf.softmax(logits, dim=1) # convert logits to probabilities
            top_p, top_class = prob.topk(1, dim = 1)

            print(f'Probability - {round(top_p.item(),2)} Class - {top_class[0].item()}')

            return top_class[0].item(), top_p.item()