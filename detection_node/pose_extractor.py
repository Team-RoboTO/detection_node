import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point
import struct
import matplotlib.pyplot as plt
import threading
import ros2_numpy as rnp 
from vision_msgs.msg import Detection2DArray


class PixelPosition(Node):

    def __init__(self):
        super().__init__('pixel_position')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.subscription_pointcloud = self.create_subscription(PointCloud2,
                                        '/zed2/zed_node/point_cloud/cloud_registered',
                                        self.pointcloud_callback, qos_profile=qos_profile)
        self.subscription_bbox = self.create_subscription(Detection2DArray,
                            '/yolo/bboxs',
                            self.subscriber_callback, qos_profile=qos_profile)
        print("nodo inizializzato")
        self.publisher = self.create_publisher(Point, 'yolo/enemy_pose', qos_profile=qos_profile)
        self.bridge = CvBridge()
        self.pointcloud = None
        self.bboxs = None

    def pointcloud_callback(self, msg):
        # Salva la point cloud come un oggetto numpy
        self.pointcloud = rnp.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=False)

    def subscriber_callback(self, msg):
        print("sottoscrivo")
        if self.pointcloud is not None:
            print("pointcloud not none")
            if len(msg.detections) != 0:
                print("someting detected")
                xc, yc=int(msg.detections[0].bbox.center.position.x),int(msg.detections[0].bbox.center.position.y)
                print(self.pointcloud.shape)
                xyz = self.pointcloud[yc-3:yc+4, xc-3:xc+4, :]
                xyz = np.nanmean(xyz, axis=(0, 1))
                point = Point()
                point.x =xyz[0]
                point.y =xyz[1]
                point.z =xyz[2]
                self.publisher.publish(point)
            
    
     

def main(args=None):
    rclpy.init(args=args)
    pixel_position = PixelPosition()
    rclpy.spin(pixel_position)
    pixel_position.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
