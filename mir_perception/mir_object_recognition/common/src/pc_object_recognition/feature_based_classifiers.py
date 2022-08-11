#!/usr/bin/env python3

import pickle
import numpy as np


class FeatureBasedClassifiers():
    """
    Defines an SVM classifier with the mean and standard deviation of
    the features, and a label encoder

    """

    def __init__(self, classifier_name, label_encoder_name):
        self.load_classifier(classifier_name, label_encoder_name)
        print('\033[92m'+"Feature based 3D object classifier model is loaded")

    def save(self, classifier_name, label_encoder_name):
        with open(classifier_name, 'wb') as f:
            pickle.dump(self.classifier, f, protocol=2)
        with open(label_encoder_name, 'wb') as f:
            pickle.dump([self.label_encoder, self.mean,
                        self.std], f, protocol=2)

    def classify(self, feature_vector, normalize=True):
        if normalize:
            feature_vector -= np.array(self.mean)
            feature_vector /= self.std

        probabilities = self.classifier.predict_proba(feature_vector)[0]
        max_index = np.argmax(probabilities)
        cls = [self.classifier.classes_[max_index]]
        return self.label_encoder.inverse_transform(cls), probabilities[max_index]

    def load_classifier(self, classifier_name, label_encoder_name):
        with open(classifier_name, 'rb') as f:
            self.classifier = pickle.load(f)
        with open(label_encoder_name, 'rb') as f:
            self.label_encoder, self.mean, self.std = pickle.load(f)
