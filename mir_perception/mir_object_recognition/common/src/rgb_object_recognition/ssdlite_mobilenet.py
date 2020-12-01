import os
import sys
import tarfile
import time
import zipfile

import numpy as np
import six.moves.urllib as urllib
import tensorflow as tf
from rgb_object_recognition.utils import util


class SSDLiteMobilenet():
    def __init__(self, checkpoint_dir): 
        # Path to frozen detection graph. This is the actual model that is used for the object detection.
        self.PATH_TO_FROZEN_GRAPH = os.path.join(checkpoint_dir, 'frozen_inference_graph.pb')
        # List of the strings that is used to add correct label for each box.
        PATH_TO_LABELS = os.path.join(checkpoint_dir, 'atwork_label_map.json')

        self.cat_idx = util.category_index_from_label_map(PATH_TO_LABELS)

        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_FROZEN_GRAPH, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session()
            # Get handles to input and output tensors
            ops = tf.get_default_graph().get_operations()
            all_tensor_names = {output.name for op in ops for output in op.outputs}
            self.tensor_dict = {}
            for key in [
                'num_detections', 'detection_boxes', 'detection_scores',
                'detection_classes'
            ]:
            tensor_name = key + ':0'
            if tensor_name in all_tensor_names:
                self.tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                    tensor_name)

            self.image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

            print ('\033[92m'+"SSDLiteMobilenet model is loaded")

    
    def infer_one_image(self, input_img):
    self.image_width = input_img.shape[1]
    self.image_height = input_img.shape[0]
    # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
    image_np_expanded = np.expand_dims(input_img, axis=0)
    # Run inference
    output_dict = self.sess.run(self.tensor_dict,
                                feed_dict={self.image_tensor: image_np_expanded})

    # all outputs are float32 numpy arrays, so convert types as appropriate
    num_detections = int(output_dict['num_detections'][0])
    det_classes = output_dict[
        'detection_classes'][0].astype(np.uint8)
    boxes = output_dict['detection_boxes'][0]
    scores = output_dict['detection_scores'][0]

    bboxes = []
    labels = []
    probs = []
    classes = []
    # Keep bboxes with scores higher that 0.5
    for i in filter(lambda id: scores[id] > 0.5, range(num_detections)):
        bboxes.append(self.convert_normalized_coordinates(boxes[i]))
        probs.append(scores[i])
        classes.append(det_classes[i])
        labels.append(self.cat_idx[det_classes[i]]['name'])

    return bboxes, probs, classes, labels

    def convert_normalized_coordinates(self, box):
    return [box[0] * self.image_height, box[1] * self.image_width, 
            box[2] * self.image_height, box[3] * self.image_width]
