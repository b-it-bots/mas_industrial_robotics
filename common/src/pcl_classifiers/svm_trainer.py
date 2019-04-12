#!/usr/bin/env python

import glob
import numpy as np
import sklearn
import sklearn.ensemble
import pcl
from mcr_object_recognition_mean_circle.features import calculate_feature_vector
from mcr_object_recognition_mean_circle.svm_classifier import SVMObjectClassifier
import colorsys


class SVMTrainer:

    def __init__(self, data_folder):
        self.data_folder = data_folder

    def train(self, objects='all'):
        objects_to_train = []
        object_directories = np.array(glob.glob(self.data_folder + '/*'))
        if objects == 'all':
            for obj_dir in object_directories:
                object_name = obj_dir.split('/')[-1]
                objects_to_train.append(str(object_name))
        else:
            objects_to_train = objects

        print "Training classifer for objects: ", objects_to_train

        n = 0
        feature_pool = np.empty([0, 0])
        label_pool = []
        for obj in objects_to_train:
            files = np.array(glob.glob(self.data_folder + '/' + obj + '/*'))
            for f in files:
                pcd = self.parse_pcd(f, True)
                features = calculate_feature_vector(pcd, True)
                if n < 1:
                    feature_pool = np.array(features)
                    label_pool = [obj]
                else:
                    feature_pool = np.vstack([feature_pool, features])
                    label_pool.append(obj)
                n += 1
        mean = np.mean(feature_pool, axis=0)
        std = np.std(feature_pool, axis=0)
        feature_pool -= mean
        feature_pool /= std

        label_encoder = sklearn.preprocessing.LabelEncoder()
        label_encoder.fit(label_pool)
        encoded_labels = label_encoder.transform(label_pool)[:, np.newaxis]
        encoded_labels = np.squeeze(encoded_labels.T)

        classifier = sklearn.ensemble.RandomForestClassifier(n_estimators=10)
        classifier.fit(feature_pool, encoded_labels)

        return SVMObjectClassifier(classifier, label_encoder, mean, std)

    def parse_pcd(self, input_file, enable_color=False):
        file_data = pcl.load(input_file).to_array()
        n_points, n_dims = file_data.shape
        if enable_color is True:
            color_table = np.float64(file_data[:, 3:])
            color_table = np.array([list(colorsys.rgb_to_hsv(i[0]/255.0, i[1]/255.0, i[2]/255.0)) for i in color_table])
            point_cloud = np.hstack([file_data[:, 0:3], color_table])
        else:
            point_cloud = file_data[:, 0:3]

        return point_cloud
