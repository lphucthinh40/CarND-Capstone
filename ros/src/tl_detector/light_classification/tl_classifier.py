from styx_msgs.msg import TrafficLight
import rospy

import tensorflow as tf
import os
import numpy as np
from collections import defaultdict
# from io import StringIO

PATH_TO_CKPT_REAL = './light_classification/models/ssd_inception_v2_real/frozen_inference_graph.pb'
PATH_TO_CKPT_SIM = './light_classification/models/ssd_inception_v2_sim/frozen_inference_graph.pb'

CATEGORY_INDEX = defaultdict(int, {1: TrafficLight.RED, 2: TrafficLight.YELLOW, 3: TrafficLight.GREEN})

CONFIDENCE_THRESHOLD = 0.6

class TLClassifier(object):
    def __init__(self, is_site):
        # Pick model based on current map
        if is_site:
        	model_path = PATH_TO_CKPT_REAL
	        rospy.loginfo("TLCLASSIFIER: Using Testing Site Model")
        else:
        	model_path = PATH_TO_CKPT_SIM
	        rospy.loginfo("TLCLASSIFIER: Using Simulation Model")

	    # Load model
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
			od_graph_def = tf.GraphDef()
			with tf.gfile.GFile(model_path, 'rb') as fid:
				serialized_graph = fid.read()
				od_graph_def.ParseFromString(serialized_graph)
				tf.import_graph_def(od_graph_def, name='')

		# Get identifiers from model graph
        with self.detection_graph.as_default():
			self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
			self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
			self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
			self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
			self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        self.sess = tf.Session(graph=self.detection_graph)

    def get_classification(self, image):
    	
    	# Add an additional dimension to satisfy model's input requirement
        image_expanded = np.expand_dims(image, axis=0)

        # Run model
        (boxes, scores, classes, num) = self.sess.run(
		  [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
		  feed_dict={self.image_tensor: image_expanded})
        
        # rospy.loginfo('class: {} \nscore: {}\n\n'.format(classes, scores))
        
        # Only cares about the best prediction (given that it passes the threshold)
        if (scores[0, 0] >= CONFIDENCE_THRESHOLD):
			light_pred = CATEGORY_INDEX[classes[0, 0]]
			return light_pred
        else:
			return TrafficLight.UNKNOWN