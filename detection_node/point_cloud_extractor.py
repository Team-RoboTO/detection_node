import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import struct

import threading

class PixelPosition(Node):

    def __init__(self):
        super().__init__('pixel_position')
        self.subscription_image = self.create_subscription(Image,
                                 '/zedm/zed_node/rgb/image_rect_color',
                                  self.image_callback, 10)
        self.subscription_pointcloud = self.create_subscription(PointCloud2,
                                        '/zedm/zed_node/point_cloud/cloud_registered',
                                        self.pointcloud_callback, 10)
        self.subscription_bbox = self.create_subscription(Float32MultiArray,
                            '/yolo/bboxs',
                            self.subscriber_callback, 10)
        self.bridge = CvBridge()
        self.pointcloud = None
        self.bboxs = None
        self.image = None
        
    def image_callback(self, msg):
        # Converti l'immagine in un oggetto numpy
        print('image callback')
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.bboxs is not None :
            for bbox in self.bboxs:
                x1, y1, x2, y2 =int(bbox[0]),int(bbox[1]),int(bbox[2]),int(bbox[3])
                xc = (x2 + x1) // 2
                yc = (y2 + y1) // 2
                xyz = self.get_pointcloud_xyz(xc,yc)
                # Estrai la distanza dal pixel desiderato dalla point cloud
            # Disegna una bounding box intorno al pixel desiderato
                print("stampo rettangolo")
                cv.rectangle(image, (x1, y1), (x2, y2), (255,0,0), 2)

                # Scrivi le coordinate sopra la bounding box
                cv.putText(image, f'({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f})', (x1, y2 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
                cv.imshow('Pixel Position', image)
        # Mostra l'immagine a video
        cv.waitKey(1)

    def pointcloud_callback(self, msg):
        # Salva la point cloud come un oggetto numpy
        self.pointcloud = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width, -1)

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
        
        if msg.layout.dim[0].label == 'full':
            msg.data.pop(-1)  #pop last value that is the delay time between two detection
            self.bboxs = np.array(msg.data, dtype=np.float32)
            self.bboxs = np.reshape(self.bboxs, (msg.layout.dim[0].size,msg.layout.dim[0].stride))

        elif msg.layout.dim[0] == 'empty':
            self.bboxs = None


 

def main(args=None):
    rclpy.init(args=args)
    pixel_position = PixelPosition()
    rclpy.spin(pixel_position)
    pixel_position.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()