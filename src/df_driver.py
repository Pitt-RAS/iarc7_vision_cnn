from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
import message_filters
import numpy as np
import rospy
from tf import transformations
import tf2_geometry_msgs
import tf2_ros
import threading

from ros_utils.make_safe_callback import make_safe_callback

from iarc7_msgs.msg import (RoombaDetection,
                            RoombaDetectionFrame)
from geometry_msgs.msg import Point32, PointStamped, Polygon, Vector3Stamped
from sensor_msgs.msg import Image, CameraInfo

from geometry import intersect_camera_with_ground


class DfDriver(object):
    def __init__(self, model, image_topic, camera_topic, camera_id):
        self._lock = threading.Lock()
        with self._lock:
            self.bridge = CvBridge()

            self._camera_id = camera_id

            self.model = model

            self.image_sub = message_filters.Subscriber(image_topic, Image)
            self.info_sub = message_filters.Subscriber(camera_topic, CameraInfo)

            self.time_synchronizer = message_filters.TimeSynchronizer(
                [self.image_sub, self.info_sub], 10)
            self.time_synchronizer.registerCallback(
                    self.camera_callback)

            self.camera_model = None

            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

            self._roomba_pub = rospy.Publisher(
                "/detected_roombas", RoombaDetectionFrame, queue_size=100)

            self._last_frame_time = rospy.Time(0)

            self._next_message = None

    def camera_callback(self, image, camera_info):
        with self._lock:
            if self.camera_model is None:
                self.camera_model = PinholeCameraModel()
                self.camera_model.fromCameraInfo(camera_info)

            self._next_message = image

    def process_next_message(self):
        with self._lock:
            if self._next_message is None:
                return

            image = self._next_message
            self._next_message = None

            self._last_frame_time = image.header.stamp

            image_bgr = self.bridge.imgmsg_to_cv2(image, 'bgr8')
            prediction = self.model.predict(image_bgr)

            try:
                optical_to_map_transform = self.tf_buffer.lookup_transform(
                        "map", image.header.frame_id, image.header.stamp, rospy.Duration(1.0))
            except Exception:
                rospy.logerr('transform failed')
                return

            roomba_frame = RoombaDetectionFrame()
            roomba_frame.header.frame_id = 'map'
            roomba_frame.header.stamp = image.header.stamp
            roomba_frame.camera_id = self._camera_id
            roomba_frame.detection_region = DfDriver.get_detection_region(
                    image, optical_to_map_transform, self.camera_model)

            for pre_dict in prediction:
                #image_width,image_height = np.shape(image_rgb)
                #Do for each prediction
                bx, by = DfDriver._bottom_midpoint_from_bounding_box(pre_dict)
                ray = DfDriver._projectTo3d(bx, by, self.camera_model)
                map_target = DfDriver._rayToXyz(
                        ray,
                        optical_to_map_transform)

                if pre_dict["label"] == "roomba":
                    roomba_frame.roombas.append(
                        DfDriver._handle_roomba_pose(map_target))
                elif pre_dict["label"] == "obstacle":
                    #TODO: write obstacle stuff
                    pass
                else:
                    raise Exception('Invalid label %s' % (pre_dict['label'], ))

            self._roomba_pub.publish(roomba_frame)

    @staticmethod
    def _bottom_midpoint_from_bounding_box(preDict):
        '''
        Takes a prediction and returns the midpoint of the bottom box
        '''
        # example bbox
        # [{"label":"roomba","confidence": 0.56,
        #         "topleft":{"x": 184, "y":101},
        #         "bottomright":{"x":274,"y:"382}}]
        # we assume that we are at least not upside down, or at a high
        # angle.... :(
        bboxWidth = preDict["bottomright"]["x"] - preDict["topleft"]["x"]
        bottomMidpoint = preDict["bottomright"]["x"] - bboxWidth / 2
        bottom = preDict["bottomright"]["y"]
        # return x,y pixels for bottom midpoint
        return [bottomMidpoint, bottom]

    @staticmethod
    def get_detection_region(image, optical_to_map_transform, camera_model):
        map_corners = []
        for px, py in [[0,           0],
                       [0,           image.height],
                       [image.width, image.height],
                       [image.width, 0]]:
            corner = DfDriver._projectTo3d(px, py, camera_model)
            normalized_corner = corner / corner[2]

            corner_vector = Vector3Stamped()
            corner_vector.vector.x = normalized_corner[0]
            corner_vector.vector.y = normalized_corner[1]
            corner_vector.vector.z = normalized_corner[2]

            map_corner_vector = tf2_geometry_msgs.do_transform_vector3(
                    corner_vector, optical_to_map_transform)

            map_corner = [
                    map_corner_vector.vector.x,
                    map_corner_vector.vector.y,
                    map_corner_vector.vector.z]
            map_corners.append(map_corner)

        camera_point = PointStamped()
        camera_point = tf2_geometry_msgs.do_transform_point(
                camera_point, optical_to_map_transform)
        camera_position = np.array([
                camera_point.point.x,
                camera_point.point.y,
                camera_point.point.z])

        polygon_points = intersect_camera_with_ground(map_corners, camera_position)

        polygon = Polygon()
        for point in polygon_points:
            point32 = Point32()
            point32.x = point[0]
            point32.y = point[1]
            point32.z = point[2]
            polygon.points.append(point32)
        return polygon

    @staticmethod
    def _projectTo3d(px, py, camera_model):
        '''
         Takes ray normal vector and the known camera transform to triangulate a
         target according to map
         '''
        result = np.asarray(camera_model.projectPixelTo3dRay((px, py)))
        assert result.shape == (3,)
        return result

    @staticmethod
    def _rayToXyz(ray, optical_to_map_transform):
        '''
        Takes a ray in the camera frame and returns a point in the map frame
        where said ray intersects the ground.  Throws ValueError if the ray
        doesn't intersect the ground
        '''
        ray = np.asarray(ray)

        quat = np.array([
            optical_to_map_transform.transform.rotation.x,
            optical_to_map_transform.transform.rotation.y,
            optical_to_map_transform.transform.rotation.z,
            optical_to_map_transform.transform.rotation.w
        ])

        optical_to_map_rot_matrix = transformations.quaternion_matrix(quat)
        optical_to_map_rot_matrix = optical_to_map_rot_matrix[:3, :3]

        map_ray = np.matmul(optical_to_map_rot_matrix, ray)

        camera_height = optical_to_map_transform.transform.translation.z

        if map_ray[2] >= 0 or camera_height < 0:
            raise ValueError('Ray doesn\'t intersect the ground')

        ray_scale = -camera_height / map_ray[2]

        assert ray_scale > 0

        map_camera_to_target = np.multiply(ray_scale, map_ray)

        return [map_camera_to_target[0]
                + optical_to_map_transform.transform.translation.x,
                map_camera_to_target[1]
                + optical_to_map_transform.transform.translation.y,
                0.0]

    @staticmethod
    def _handle_roomba_pose(position):
        roomba = RoombaDetection()
        roomba.pose.x = position[0]
        roomba.pose.y = position[1]
        roomba.pose.theta = 0

        roomba.position_covariance = [0, 0, 0, 0]
        roomba.box_uncertainty = np.finfo(np.float32).max
        roomba.flip_certainty = 0
        return roomba
