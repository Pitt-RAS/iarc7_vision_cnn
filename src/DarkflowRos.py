#!/usr/bin/env python2

from darkflow.net.build import TFNet
from df_driver import DfDriver

import rospy
import threading

class DfModel(object):
    '''
    creates a DF model with the following methods:

    predict(image): returns prediction for a [M,N,3] array

    '''

    def __init__(self, options):
        self._lock = threading.Lock()
        with self._lock:
            if not options:
                raise Exception("DarkFlow options not set")
            else:
                self.model = TFNet(options)

    def predict(self, image):
        with self._lock:
            if image is None:
                raise Exception('Predict received `None` image')
            return self.model.return_predict(image)

if __name__ == '__main__':
    rospy.init_node('darkflow_ros')

    # obstacle_pub = rospy.Publisher('/detected_obstacles', ObstacleArray,queue_size=5)

    namespaces = rospy.get_param('~namespaces')
    image_topics = [namespace + 'image_raw' for namespace in namespaces]
    camera_topics = [namespace + 'camera_info' for namespace in namespaces]

    options = {
        "model":      rospy.get_param('~config_path'),
        "backup":     rospy.get_param('~checkpoint_path'),
        "load":       rospy.get_param('~checkpoint'),
        "gpu":        rospy.get_param('~gpu_usage'),
        "threshold":  rospy.get_param('~detection_threshold'),
        "labels":     rospy.get_param('~labels_path')
    }

    model = DfModel(options)

    dfs = []
    for image_topic, camera_topic in zip(image_topics, camera_topics):
        dfs.append(DfDriver(model, image_topic, camera_topic, image_topic))
    rospy.spin()
