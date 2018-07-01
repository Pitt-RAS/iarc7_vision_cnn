#!/usr/bin/env python2

from darkflow.net.build import TFNet
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel

class DfModel:
    '''
    creates a DF model with the following methods:

    predict(image): returns prediction for a [M,N,3] array

    '''
    def __init__(self,options):
#        self.options = {"model": "./cfg/tiny-yolo-udacity.cfg",\ 
#                "backup": "./ckpt/","load": 8987, "gpu": 1.0
#                "threshold": 0.1 "labels": "./labels.txt"}

        if not options:
            rospy.logerr("DarkFlow options not set")
        else:
            self.options = options

            self.model = TFNet(self.options)


    def predict(self,image):
        if image is not None:
            prediction = self.model.return_predict([image])
        else:
            rospy.logerr("Image could not be send to DarkFlow")
            prediction = None
        return prediction
        

class DfDriver:
    def __init__(self,options):
        self.options = options

        self.model = DfModel(self.options)

        # camera data
        rospy.Subscriber("image0", Image, self.image_callback,\
        queue_size = 1)

        self.cameraInfoSub = rospy.Subscriber("itopic0", CameraInfo,\
        self.itopic_callbajk)

        self.camera = PinholeCameraModel()

        self.prediction = None

    def image_callback(self, image):
        image_rgb = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        self.prediction = self.model.predict(image_rgb)    


    def itopic_callback(self, itopic):
        camera.fromCameraInfo(itopic)
        self.cameraInfoSub.unregister()

if __name__ == '__main__':

    rospy.init_node('darkflow_ros')
    th   = rospy.get_param('~threshold')
    cpth = rospy.get_param('~checkpoint_path')
    ckpt = rospy.get_param('~checkpoint')
    cfg  = rospy.get_param('~config_path')
    gpu  = rospy.get_param('~gpu_usage')
    lbls = rospy.get_param('~labels_path')

    options = {"model": cfg, "backup": cpth, \
                "load": ckpt, "gpu": gpu, "threshold": th,\
                "labels": lbls}



    df = DfDriver(options)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        print(df.prediction)
        rate.sleep()

    # get images from stream, and classify
