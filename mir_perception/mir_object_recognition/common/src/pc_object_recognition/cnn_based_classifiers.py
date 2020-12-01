from abc import ABCMeta, abstractmethod

class CNNBasedClassifiers(object):
    """
    Abstraction class for CNN based classifiers
    """

    __metaclass__ = ABCMeta

    def __init__(self, **kwargs):
        self.checkpoint_path = kwargs.get("checkpoint_path", None)
        self.num_classes = kwargs.get("num_classes", None)
        self.num_points = kwargs.get("num_points", None)
        self.cloud_dim = kwargs.get("cloud_dim", None)

    @abstractmethod
    def classify(self, pointcloud):
        """
        A classify method to be implemented

        :param pointcloud:    The input pointcloud
        :type:                numpy.array

        :return:            Predicted label and probability
        """
        pass