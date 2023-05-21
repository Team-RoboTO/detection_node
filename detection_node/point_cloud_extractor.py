import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import struct
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import threading
import ros2_numpy as rnp

class PixelPosition(Node):

    def __init__(self):
        super().__init__('pixel_position')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.subscription_image = self.create_subscription(Image,
                                 '/zed2/zed_node/rgb/image_rect_color',
                                  self.image_callback, qos_profile=qos_profile)
        self.subscription_pointcloud = self.create_subscription(PointCloud2,
                                        '/zed2/zed_node/point_cloud/cloud_registered',
                                        self.pointcloud_callback, qos_profile=qos_profile)
        self.subscription_bbox = self.create_subscription(Image,
                            '/yolo/bboxs',
                            self.subscriber_callback, qos_profile=qos_profile)
        self.bridge = CvBridge()
        self.pointcloud = None
        self.bboxs = None
        self.image = None
        
    def image_callback(self, msg):
        # Converti l'immagine in un oggetto numpy
        print('image callback')
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.bboxs.size != 0:
            for bbox in self.bboxs:
                x1, y1, x2, y2, id =int(bbox[0]),int(bbox[1]),int(bbox[2]),int(bbox[3]), nt(bbox[4])
                xc = (x2 + x1) // 2
                yc = (y2 + y1) // 2
                xyz = self.get_pointcloud_xyz(xc,yc)
                cv.rectangle(image, (x1, y1), (x2, y2), (255,0,0), 2)
                cv.putText(image, f'({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f})', (x1, y2 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
                cv.imshow('Pixel Position', image)
            else:
                cv.imshow('Pixel Position', image)
        # Mostra l'immagine a video
        cv.waitKey(1)

    def pointcloud_callback(self, msg):
        # Salva la point cloud come un oggetto numpy
        self.pointcloud = rnp.point_cloud2.get_xyz_points(rnp.point_cloud2.pointcloud2_to_array(msg), remove_nans=False)

    def get_pointcloud_xyz(self, x, y):
        # Controlla se la point cloud è stata definita
        if self.pointcloud is not None:
            # Estrai le coordinate XYZ dal pixel desiderato dalla point cloud
            xyz = self.pointcloud[y-3:y+4, x-3:x+4, :3]
            xyz_mean = np.nanmean(xyz, axis=(0, 1))
            return xyz_mean
        else:
            # Ritorna un valore di default se la point cloud non è stata ancora definita
            return np.array([0, 0, 0])

    def subscriber_callback(self, msg):
        self.bboxs = rnp.numpify(msg)


 

def main(args=None):
    rclpy.init(args=args)
    pixel_position = PixelPosition()
    rclpy.spin(pixel_position)
    pixel_position.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()