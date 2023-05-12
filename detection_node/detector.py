#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import time
from std_msgs.msg import Float32MultiArray, Float64MultiArray, MultiArrayDimension
import numpy as np
import pdb
import torch
import time
from time import sleep
from threading import Thread, Lock

class DetectorAndTracker(Node):
    def __init__(self):
        super().__init__("DetectorAndTracker")
        self.bridge = CvBridge()
        self.rgb_subscriber = self.create_subscription(Image,
                                "/zedm/zed_node/rgb/image_rect_color",
                                self.run, 10)
        self.publisher = self.create_publisher(Float32MultiArray, 'yolo/bboxs', 10)
          # This will automatically download the trained model
        self.frame = None
        self.tt = -1
        self.model = YOLO("/home/auto/Downloads/best_armor.engine", task='detect')
        print("yolo is loaded...")

    def run(self, frame):
        #self.lock.acquire()
        self.frame = self.bridge.imgmsg_to_cv2(frame, "bgr8") 
        prima = time.time() # Convert the image to np.array
        results = self.model.track(self.frame,classes=0, device=0)  # Compute predictions
        t = time.time()
        print("detection time", prima-t)
        if self.tt !=-1:
            delay = t - self.tt
        else:
            delay = -1
        self.tt = t
        print("delay from previous spin", delay)
        boxes = results[0].boxes.xyxy.cpu().numpy()
        self.publish_numpy_array(boxes, delay)
        #self.lock.release()

    def publish_numpy_array(self, numpy_array, delay):
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        if numpy_array.size != 0:
            dim.label= 'full'
            dim.stride = 4
            dim.size = numpy_array.shape[0]
            msg.layout.dim.append(dim)
            msg.data = numpy_array.flatten().tolist()
            msg.data.append(delay) #push delay value between two detection at the end of the buffer
        else:
            dim.label= 'empty'
            dim.stride = 0
            dim.size = 0
            msg.layout.dim.append(dim)
            msg.data = [0.0]

        self.publisher.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    print("start")
    detector= DetectorAndTracker()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
