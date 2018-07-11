#!/usr/bin/env python2

from darkflow.net.build import TFNet
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs import msg
from tf import transformations
import tf_conversions
import geometry_msgs.msg


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
    # camera callbacks
    def image_callback(self, image):
        image_rgb = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        self.prediction = self.model.predict(image_rgb)    
        if self.prediction:
        #image_width,image_height = np.shape(image_rgb)
            #Do for each prediction
            n = 0
            bx,by = self._bboxToGP(self.prediction[n])
            ray = self._projectTo3d(bx,by)
            target = self._rayToXyz(ray,self.camera_info.header.frame_id,\
                    self.camera_info.header.stamp)
            print "target",target 
            m = [None]*3
            m[0] = target[0]
            m[1] = target[1]
            m[2] = target[2]
            self._handle_target_pose(m,"roomba")

    def camera_callback(self, camera_info):
#        self.camera_frame = camera_info.header.frame_id
#        self.camera_stamp = camera_info.header.stamp
        self.camera_info = camera_info
        self.camera_model.fromCameraInfo(camera_info)

################End Camera 1 Stuff########################################

    def _bboxToGP(self,preDict):

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
        bottom = preDict["bottomright"]["y"]
        # return x,y pixels for bottom midpoint
        return [bottomMidpoint,bottom]

    def _pixelToRay(self,px,py,pw,ph):
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

    def _projectTo3d(self,px,py):
        '''
         Takes ray normal vector and the known camera transform to triangulate a 
         target according to map
         '''

        [vx,vy,vz] = self.camera_model.projectPixelTo3dRay((px,py))
        #_z = self._getDepthAt(x,y)
        #_x = vx * _z
        #_y = vy * _z
        ray = [None]*3
        ray[0] = vx
        ray[1] = vy
        ray[2] = vz

#        norm = np.sqrt(ray[0]**2 + ray[1]**2 + ray[2]**2)

#        ray[0] /= norm
#        ray[1] /= norm
#        ray[2] /= norm

        return ray

    def _rayToXyz(self,ray,frame,stamp):
        ray = np.asarray(ray)
        print "ray",ray
         
        o_to_m_trans = tf_buffer.lookup_transform("map",frame,stamp)

        quat = np.array([o_to_m_trans.transform.rotation.x,
            o_to_m_trans.transform.rotation.y,
            o_to_m_trans.transform.rotation.z,
            o_to_m_trans.transform.rotation.w])

        o_to_m_rot_matrix = transformations.quaternion_matrix(quat)
        o_to_m_rot_matrix = o_to_m_rot_matrix[:3,:3]

        map_ray = np.matmul(o_to_m_rot_matrix,ray)

        camera_height = o_to_m_trans.transform.translation.z

        ray_scale = -camera_height/map_ray[2]
        print "ray_scale",ray_scale

        Ray = np.multiply(ray_scale,map_ray)

        target = [None]*3
        print "Ray",Ray
        target[0] = Ray[0] + o_to_m_trans.transform.translation.x
        target[1] = Ray[1] + o_to_m_trans.transform.translation.y
#        target[2] = Ray[2] + o_to_m_trans.transform.translation.z
        target[2] = 0 #roombas are always on the floor

        return target

    def _handle_target_pose(self,m,label):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = label
        t.transform.translation.x = m[0]
        t.transform.translation.y = m[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1
        br.sendTransform(t)



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

    tf_buffer = tf2_ros.Buffer() #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        print(df.prediction)
        rate.sleep()

    # get images from stream, and classify
