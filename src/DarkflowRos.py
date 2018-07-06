#!/usr/bin/env python2

from darkflow.net.build import TFNet
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
import tf
import tf2_ros
from geometry_msgs import msg

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
            prediction = self.model.return_predict(image)
        else:
            rospy.logerr("Image could not be send to DarkFlow")
            prediction = None
        return prediction
        

class DfDriver:
    def __init__(self,options):

        # minor housekeeping first
        self.options = options

        self.model = DfModel(self.options)

        self.bridge = CvBridge()

################Camera 1 Stuff############################################
        # camera data
        rospy.Subscriber("camera", Image, self.image_callback,\
        queue_size = 1)

        self.cameraInfoSub = rospy.Subscriber("camera_topic", CameraInfo,\
        self.camera_callback)

        self.camera_model = PinholeCameraModel()

        self.prediction = None

#        self.cameraAov = rospy.get_param('roomba_estimator_settings/%s_aov'%)
        self.cameraAov = rospy.get_param('roomba_estimator/front_camera_aov')

        self.camera_frame = camera_info

    # camera callbacks
    def image_callback(self, image):
        image_rgb = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        self.prediction = self.model.predict(image_rgb)    

    def camera_callback(self, camera_info):
        self.camera_model.fromCameraInfo(camera_info)
        self.cameraInfoSub.unregister()
        
################End Camera 1 Stuff########################################
    def bboxToGP(self,preDict):

        '''
        Takes a prediction and returns the midpoint of the bottom box
        '''
       # example bbox
       # [{"label":"roomba","confidence": 0.56,
       #         "topleft":{"x": 184, "y":101},
       #         "bottomright":{"x":274,"y:"382}}]
       # we assume that we are at least not upside down, or at a high
       # angle.... :(
#       bboxHeight = preDict["bottomright"]["y"] - preDict["topleft"]["y"]
       bboxWidth = preDict["bottomright"]["x"] - preDict["topleft"]["x"]
       bottomMidpoint = preDict["bottomright"]["x"] - bboxWidth / 2
       # return x,y pixels for bottom midpoint
       return [bottomMidpoint,preDict["bottomright"]["y"]]
        
        
    
    def pixelToRay(self,px,py,pw,ph):
        '''
        Takes pixel coordinates at px,py and image dimension width: pw and
        height ph, returns a normal vector from pixel
        '''

        px -= pw * 0.5
        py -= ph * 0.5

        pix_r = np.hypot(px,py)
        pix_R = np.hypot(ph,pw) * 0.5

        max_phi = self.cameraAov * np.pi / 360
        pix_focal = pix_R / np.tan(max_phi)
        theta = atan2(py, px)

        camera_radius = pix_r / pix_focal

        # x = cam_radius*cos(theta), y = cam_radius*sin(theta), z = 1
        ray = [camera_radius*np.cos(theta),camera_radius*sin(theta),1]

        norm = np.sqrt(ray[0]**2 + ray[1]**2 + ray[2]**2)

        ray[0] /= norm
        ray[1] /= norm
        ray[2] /= norm

        return ray

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
