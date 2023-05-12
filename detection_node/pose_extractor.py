import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import struct
import matplotlib.pyplot as plt
import threading

class PixelPosition(Node):

    def __init__(self):
        super().__init__('pixel_position')
        self.subscription_pointcloud = self.create_subscription(PointCloud2,
                                        '/zedm/zed_node/point_cloud/cloud_registered',
                                        self.pointcloud_callback, 10)
        self.subscription_bbox = self.create_subscription(Float32MultiArray,
                            '/yolo/bboxs',
                            self.subscriber_callback, 10)
        self.publisher = self.create_publisher(Float32MultiArray, 'yolo/enemy_pose', 10)
        self.bridge = CvBridge()
        self.pointcloud = None
        self.bboxs = None
        self.figx, self.ax = plt.subplots()
        self.figy, self.ay = plt.subplots()
        self.figz, self.az = plt.subplots()
        self.x = []
        self.y = []
        self.z = []

    def pointcloud_callback(self, msg):
        # Salva la point cloud come un oggetto numpy
        self.pointcloud = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width, -1)

    def subscriber_callback(self, msg):
            if msg.layout.dim[0].label == 'full':
                delay = msg.data[-1]
                msg.data.pop(-1)  #pop last value that is the delay time between two detection
                self.bboxs = np.array(msg.data, dtype=np.float32)
                self.bboxs = np.reshape(self.bboxs, (msg.layout.dim[0].size,msg.layout.dim[0].stride))
                bbox = self.bboxs[-1, :]
                x1, y1, x2, y2 =int(bbox[0]),int(bbox[1]),int(bbox[2]),int(bbox[3])
                xc = (x2 + x1) // 2
                yc = (y2 + y1) // 2
                xyz = self.pointcloud[y-3:y+4, x-3:x+4, :3]
                xyz = np.nanmean(xyz, axis=(0, 1))
                # self.x.append(xyz[0])
                # self.y.append(xyz[1])
                # self.z.append(xyz[2])
                # print(xyz)
                msg = Float32MultiArray()
                dim = MultiArrayDimension()
                if xyz.size != 0:
                    dim.label= 'full'
                    dim.stride = 3
                    dim.size = 1
                    msg.layout.dim.append(dim)
                    msg.data = xyz.tolist()
                    msg.data.append(delay)
                else:
                    dim.label= 'empty'
                    dim.stride = 0
                    dim.size = 0
                    msg.layout.dim.append(dim)
                    msg.data = [0.0]
                self.publisher.publish(msg)
                # self.ax.clear()
                # self.ax.plot(self.x)
                # self.ay.clear()
                # self.ay.plot(self.y)
                # self.az.clear()
                # self.az.plot(self.z)
                # self.figx.canvas.draw()
                # self.figy.canvas.draw()
                # self.figz.canvas.draw()
                # plt.pause(0.0001)
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