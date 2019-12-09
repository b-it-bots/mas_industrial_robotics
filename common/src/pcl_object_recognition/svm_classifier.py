#!/usr/bin/env python

import pickle
import numpy as np


class SVMObjectClassifier:
    """
    Defines an SVM classifier with the mean and standard deviation of
    the features, and a label encoder

    """
    def __init__(self, classifier, label_encoder, mean, std):
        self.classifier = classifier
        self.label_encoder = label_encoder
        self.mean = mean
        self.std = std

    def save(self, classifier_name, label_encoder_name):
        with open(classifier_name, 'wb') as f:
            pickle.dump(self.classifier, f, protocol=2)
        with open(label_encoder_name, 'wb') as f:
            pickle.dump([self.label_encoder, self.mean, self.std], f, protocol=2)

    def classify(self, feature_vector):
        #feature_vector -= np.array(self.mean)
        #feature_vector /= self.std
        probabilities = self.classifier.predict_proba(feature_vector)[0]
        max_index = np.argmax(probabilities)
        cls = [self.classifier.classes_[max_index]]
        return self.label_encoder.inverse_transform(cls), probabilities[max_index]

    @classmethod
    def load(cls, classifier_name, label_encoder_name):
        print "Classifier name: ", classifier_name
        with open(classifier_name, 'rb') as f:
            classifier = pickle.load(f)
        with open(label_encoder_name, 'rb') as f:
            [label_encoder, mean, std] = pickle.load(f)
        return SVMObjectClassifier(classifier, label_encoder, mean, std)
