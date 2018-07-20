#!/usr/bin/env python2

from darkflow.net.build import TFNet
from df_driver import DfDriver

import rospy

class DfModel(object):
    '''
    creates a DF model with the following methods:

    predict(image): returns prediction for a [M,N,3] array

    '''

    def __init__(self, options):
        if not options:
            raise Exception("DarkFlow options not set")
        else:
            self.model = TFNet(options)

    def predict(self, image):
        if image is None:
            raise Exception('Predict received `None` image')
        return self.model.return_predict(image)


if __name__ == '__main__':
    rospy.init_node('darkflow_ros')

    # obstacle_pub = rospy.Publisher('/detected_obstacles', ObstacleArray,queue_size=5)

    image_topic = rospy.get_param('~image_topic')
    camera_topic = rospy.get_param('~camera_topic')

    options = {
        "model":      rospy.get_param('~config_path'),
        "backup":     rospy.get_param('~checkpoint_path'),
        "load":       rospy.get_param('~checkpoint'),
        "gpu":        rospy.get_param('~gpu_usage'),
        "threshold":  rospy.get_param('~detection_threshold'),
        "labels":     rospy.get_param('~labels_path')
    }

    model = DfModel(options)

    df = DfDriver(model, image_topic, camera_topic, image_topic)
    rospy.spin()
