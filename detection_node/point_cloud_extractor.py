import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import struct


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
        #self.sync = message_filters.ApproximateTimeSynchronizer([self.subscription_bbox, self.subscription_pointcloud, self.subscription_image], 10, 0.1, allow_headerless=True)
        self.bridge = CvBridge()
        self.pointcloud = None
        self.bboxs = None
        
    def image_callback(self, msg):
        # Converti l'immagine in un oggetto numpy
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.bboxs is not None:
            for bbox in self.bboxs:
                x1, y1, x2, y2 =int(bbox[0]),int(bbox[1]),int(bbox[2]),int(bbox[3])
                xc = (x2 + x1) // 2
                yc = (y2 + y1) // 2
                xyz = self.get_pointcloud_xyz(xc,yc)
                # Estrai la distanza dal pixel desiderato dalla point cloud
            # Disegna una bounding box intorno al pixel desiderato
            
                cv2.rectangle(image, (x1, y1), (x2, y2), (255,0,0), 2)

                # Scrivi le coordinate sopra la bounding box
                cv2.putText(image, f'({xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f})', (x1, y2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

            # Mostra l'immagine a video
            i=0
        cv2.imshow('Pixel Position', image)
        cv2.waitKey(1)

    def pointcloud_callback(self, msg):
        # Salva la point cloud come un oggetto numpy
        self.pointcloud = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width, -1)

    def get_pointcloud_xyz(self, x, y):
        # Controlla se la point cloud è stata definita
        if self.pointcloud is not None:
            # Estrai le coordinate XYZ dal pixel desiderato dalla point cloud
            xyz = self.pointcloud[y, x, :]
            return xyz
        else:
            # Ritorna un valore di default se la point cloud non è stata ancora definita
            return np.array([0, 0, 0])

    def subscriber_callback(self, msg):
        
        if msg.layout.dim[0].label == 'full':
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