#!/usr/bin/env python

import glob
import numpy as np
import sklearn
import sklearn.ensemble
import pcl
from features import calculate_feature_vector, calculate_maxrdd_fv_features
from svm_classifier import SVMObjectClassifier
import colorsys
import os
import gzip
import pickle


class SVMTrainer:

    def __init__(self, data_folder):
        self.data_folder = data_folder

    def load_compressed_pickle_file(self, pickle_file_name):
        with gzip.open(pickle_file_name, 'rb') as f:
            return pickle.load(f)

    def train(self, data_folder, objects='all',gmm=None, feature_extraction='msc_fv_3', mc_feature=True):
        objects_to_train = []
        object_directories = np.array(glob.glob(data_folder + '/*'))
        if objects == 'all':
            for obj_dir in object_directories:
                print (obj_dir)
                object_name = obj_dir.split('/')[-1]
                objects_to_train.append(str(object_name))
        else:
            objects_to_train = objects

        print ("Extracting features....")
        n = 0
        feature_pool = np.empty([0, 0])
        label_pool = []
        for obj in objects_to_train:
            files = np.array(glob.glob(data_folder + '/' + obj + '/*'))
            print (obj, ": ", len(files))
            for f in files:
                points = self.parse_pcd(f, enable_color=True)
                if feature_extraction == 'mc':
                    features = calculate_feature_vector(points, enable_color=True)
                else:
                    features = calculate_maxrdd_fv_features(points, gmm, feature_extraction, mean_circle=mc_feature)
                
                if n < 1:
                    feature_pool = np.array(features)
                    label_pool = [obj]
                else:
                    feature_pool = np.vstack([feature_pool, features])
                    label_pool.append(obj)
                n += 1

        mean = np.mean(feature_pool, axis=0)
        std = np.std(feature_pool, axis=0)
        #feature_pool -= mean
        #feature_pool /= std

        feature_pool = np.nan_to_num(feature_pool)
        print ("Classifying....", feature_pool.shape)

        label_encoder = sklearn.preprocessing.LabelEncoder()
        label_encoder.fit(label_pool)
        encoded_labels = label_encoder.transform(label_pool)[:, np.newaxis]
        encoded_labels = np.squeeze(encoded_labels.T)

        classifier = sklearn.ensemble.RandomForestClassifier(n_estimators=10)
        classifier.fit(feature_pool, encoded_labels)

        return SVMObjectClassifier(classifier, label_encoder, mean, std)

    def train_using_pgz(self, data_folder, gmm, feature_extraction='msc_fv_3', objects='all', datasize=2):
        data = None
        labels = None
        count = 0
        for i in range(datasize):
            print ("Processing: {}data{}.pgz".format(data_folder,i))
            dataset = self.load_compressed_pickle_file('{}data{}.pgz'.format(data_folder,i))
            if i == 0:
                data = dataset['data']
                labels = dataset['labels']
            else:
                data = np.concatenate((data, dataset['data']))
                labels = np.concatenate((labels, dataset['labels']))
        
        print ("Computing features for datasize: {}".format(data.shape))
        print ("Label shape: ", labels.shape)
        n = 0
        feature_pool = np.empty([0, 0])
        label_pool = []
        for i,points in enumerate(data):
            if feature_extraction == 'mc':
                features = calculate_feature_vector(points)
            else:
                features = calculate_maxrdd_fv_features(points, gmm, feature_extraction, mean_circle=False)
            if i < 1:
                #if not np.isnan(features.any()):
                feature_pool = np.array(features)
                label_pool = [labels[i]]
            else:
                #if not np.isnan(features.any()):
                feature_pool = np.vstack([feature_pool, features])
                label_pool.append(labels[i])

        print ("Done features computation")
        
        # Whitenin, and remember to whiten the input for inference
        mean = np.mean(feature_pool, axis=0)
        std = np.std(feature_pool, axis=0)
        #feature_pool -= mean
        #feature_pool /= std
        feature_pool = np.nan_to_num(feature_pool)
        print ("Classifying....", feature_pool.shape)
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
