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
        super().__init__("image_subscriber")
        self.bridge = CvBridge()
        self.rgb_subscriber = self.create_subscription(Image,
                                "/zedm/zed_node/rgb/image_rect_color",
                                self.run, 5)
        self.publisher = self.create_publisher(Float32MultiArray, 'yolo/bboxs', 10)
          # This will automatically download the trained model
        self.frame = None
        self.lock = Lock()

    def run(self, frame):
        self.lock.acquire()
        self.frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")  # Convert the image to np.array
        self.lock.release()

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
    def get_frame(self):

        return self.frame
    
    def clear(self):
        self.frame = None
    def get_lock(self):
        return self.lock
    

def yolo_func(node):
    model = YOLO("yolov8n.engine", task='detect')
    print("yolo is loaded...")
    lock = node.get_lock()
    tt = -1
    while True:
        lock.acquire()
        frame = node.get_frame()
        
        prima = time.time()
        results = model.track(frame,classes=0, tracker="bytetrack.yaml", device=0)  # Compute predictions
        t = time.time()
        lock.release()
        
        print("detection time", prima-t)
        if tt !=-1:
            delay = t - tt
        else:
            delay = -1
        tt = t
        print("delay from previous spin", delay)
        #out = results[0].plot()  # This plots the prediction on the original image
        # Publish after converting to ROS format
        boxes = results[0].boxes.xyxy.cpu().numpy()
        node.publish_numpy_array(boxes, delay)
        #self.detector_publisher.publish(self.bridge.cv2_to_imgmsg(out, "rgb8"))


def main(args=None):
    rclpy.init(args=args)
    print("start")
    detector= DetectorAndTracker()
    yolo = Thread(target=yolo_func, kwargs={'node':detector})
    print("launch...")
    yolo.start()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
