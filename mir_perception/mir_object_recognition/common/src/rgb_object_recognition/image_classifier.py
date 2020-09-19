from abc import ABCMeta, abstractmethod

class ImageClassifier(object):
    """
    Abstraction class for image classifiers
    """

    __metaclass__ = ABCMeta

    def __init__(self, **kwargs):
        self.checkpoint_path = kwargs.get("checkpoint_path", None)

    @abstractmethod
    def classify(self, image):
        """
        A classify method to be implemented

        :param image:  The input rgb image
        :type name:         numpy.array

        :return:  bounding boxes, probabilities and classes
        """
        pass