import rospy
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import Marker
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import yaml
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from threading import Lock
import os
import rospkg

class EmptySpaceDetector:
    def __init__(self):
        rospy.init_node("empty_space_detector")

        self.event_sub = rospy.Subscriber("/empty_space_detector/event_in", String, self.event_callback)
        self.event_pub = rospy.Publisher("/empty_space_detector/event_out", String, queue_size=10)

        self.process_image = False
        self.process_pointcloud = False  # Corrected typo

        self.lock = Lock()

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('mir_empty_space_prediction')

        self.model_path = os.path.join(package_path, "model/best_empty.pt") # updating new model

        # Load ROI parameters of all WS
        roi_params_path = os.path.join(package_path, "config/params.yaml")

        with open(roi_params_path, 'r') as file:
            all_roi_params = yaml.safe_load(file)
        
        # get worsktation from ros param
        self.workstation = rospy.get_param("/place_object_server/worskstation")
        print(self.workstation)

        if self.workstation in all_roi_params:
            roi=all_roi_params[self.workstation]
            print(roi)
        else:
            rospy.logwarn(f"Invalid workstation '{self.workstation}', defaulting to WS01")
            roi = all_roi_params["DEFAULT"]

        # with open(roi_params_path, 'r') as file:
        #     roi_params = yaml.safe_load(file)

        self.roi = (roi['x_min'], roi['x_max'], roi['y_min'], roi['y_max'])


        # self.roi = (roi_params['x_min'], roi_params['x_max'], roi_params['y_min'], roi_params['y_max'])

        self.model = YOLO(self.model_path)
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.center = None
        self.best_box = None
        
        rospy.Subscriber("/tower_cam3d_front/color/image_raw", Image, self.image_callback)
        rospy.Subscriber("/tower_cam3d_front/depth/color/points", PointCloud2, self.pointcloud_callback)
        self.marker_pub = rospy.Publisher("/projected_point_marker", Marker, queue_size=10)
        self.debug_image_pub = rospy.Publisher("/debug_empty_space", Image, queue_size=10)
        self.empty_space_pub = rospy.Publisher("/empty_space_pose", PoseStamped, queue_size=10)

        self.predictions_pub = rospy.Publisher("/empty_space_predictions", Image, queue_size=10)


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def event_callback(self, msg):
        with self.lock:
            if msg.data == "e_empty":
                self.process_image = True
                self.process_pointcloud = False
            elif msg.data == "e_cloud":
                self.process_pointcloud = True
                self.process_image = False
            elif msg.data == "e_stop":
                self.process_image = False
                self.process_pointcloud = False
                self.center = None
                rospy.loginfo("Received stop event. Processing halted.")
    
    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            if self.process_image:
                self.predict_empty_space()
    
    def pointcloud_callback(self, msg):
        with self.lock:
            if self.process_pointcloud and self.center is not None:
                self.process_pointcloud_data(msg)
    
    def predict_empty_space(self):
        if self.latest_image is None:
            return
        
        results = self.model(self.latest_image)
        boxes = results[0].boxes.xyxy.cpu().numpy()

        #publish all predictions 
        self.publish_all_predictions(boxes)
        
        roi_boxes = [box for box in boxes if self.is_box_in_roi(box)]
        if not roi_boxes:
            self.center = None
            self.best_box = None
            return
        
        self.best_box = max(roi_boxes, key=lambda box: self.box_area(box))
        self.center = self.box_center(self.best_box)
        rospy.loginfo(f"Best empty space center: {self.center}")
        
        self.publish_debug_image()
        
        if self.center is not None:
            self.event_pub.publish(String("e_empty_space_detected"))
        else:
            self.event_pub.publish(String("e_no_empty_space_detected"))

    
    def is_box_in_roi(self, box):
        return (box[0] >= self.roi[0] and box[2] <= self.roi[1] and
                box[1] >= self.roi[2] and box[3] <= self.roi[3])
    
    def box_area(self, box):
        return (box[2] - box[0]) * (box[3] - box[1])
    
    def box_center(self, box):
        return ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)
    
    def publish_debug_image(self):
        if self.latest_image is None or self.best_box is None:
            return
        
        debug_image = self.latest_image.copy()
        
        # Draw ROI
        cv2.rectangle(debug_image, (self.roi[0], self.roi[2]), (self.roi[1], self.roi[3]), (0, 0, 255), 2)
        
        # Draw best box
        cv2.rectangle(debug_image, 
                      (int(self.best_box[0]), int(self.best_box[1])), 
                      (int(self.best_box[2]), int(self.best_box[3])), 
                      (0, 255, 0), 2)
        
        # Draw center point
        cv2.circle(debug_image, (int(self.center[0]), int(self.center[1])), 5, (255, 0, 0), -1)
        
        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
        self.debug_image_pub.publish(debug_msg)

    
    def publish_all_predictions(self, boxes):
        if self.latest_image is None:
            rospy.logwarn("No image available for visualization.")
            return

        debug_image = self.latest_image.copy()

        if not boxes.any():
            rospy.logwarn("No empty space predictions found.")
        else:
            for box in boxes:
                # Draw each bounding box in green
                cv2.rectangle(debug_image, 
                            (int(box[0]), int(box[1])), 
                            (int(box[2]), int(box[3])), 
                            (0, 255, 0), 2)
        # Draw ROI
        cv2.rectangle(debug_image, (self.roi[0], self.roi[2]), (self.roi[1], self.roi[3]), (0, 0, 255), 2)

        # Convert OpenCV image to ROS Image message
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")

        # Publish the debug image with all predictions
        self.predictions_pub.publish(debug_msg)


    def project_pixel_to_pointcloud(self, cloud_msg, pixel_x, pixel_y):
        width = cloud_msg.width
        height = cloud_msg.height

        if width <= 1 or height <= 1:
            rospy.logerr("PointCloud is unordered. Cannot directly map pixel coordinates.")
            return None

        points_list = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=False))
        points_array = np.array(points_list).reshape(height, width, 3)
        reshaped_points = np.zeros((480, 640, 3))

        for i in range(3):
            reshaped_points[:,:,i] = cv2.resize(points_array[:,:,i], (640, 480), interpolation=cv2.INTER_LINEAR)

        return reshaped_points[int(pixel_y), int(pixel_x)]

    def process_pointcloud_data(self, cloud_msg):
        if self.center is None:
            return

        pixel_x, pixel_y = self.center
        projected_point = self.project_pixel_to_pointcloud(cloud_msg, pixel_x, pixel_y)

        if projected_point is not None and not np.isnan(projected_point).any():
            x, y, z = projected_point
            # rospy.loginfo(f"Projected 3D Point: x={x}, y={y}, z={z}")

            marker = Marker()
            marker.header.frame_id = cloud_msg.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "projected_point"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.a = 1.0

            self.marker_pub.publish(marker)

            pose_stamp = PoseStamped()
            pose_stamp.header.frame_id = cloud_msg.header.frame_id
            pose_stamp.header.stamp = rospy.Time.now()
            pose_stamp.pose.position.x = x
            pose_stamp.pose.position.y = y
            pose_stamp.pose.position.z = z
            pose_stamp.pose.orientation.w = 1.0

            # publishing in same camera frame
            self.empty_space_pub.publish(pose_stamp)
            # rospy.loginfo("Published empty space pose and orientation in camera frame : " + str(pose_stamp.pose.position))
            self.event_pub.publish(String("e_pointcloud_processed"))
        else:
            rospy.logwarn("No valid point found for the given pixel coordinates.")

if __name__ == "__main__":
    detector = EmptySpaceDetector()
    rospy.spin()
