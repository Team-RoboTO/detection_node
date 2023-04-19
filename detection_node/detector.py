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

class DetectorAndTracker(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.bridge = CvBridge()
        self.rgb_subscriber = self.create_subscription(Image,
                                "/zedm/zed_node/rgb/image_rect_color",
                                self.run, 5)
        self.publisher = self.create_publisher(Float32MultiArray, 'yolo/bboxs', 5)
        self.model = YOLO("yolov8n.pt")  # This will automatically download the trained model


    def run(self, frame):
        try:

            frame = self.bridge.imgmsg_to_cv2(frame, "rgb8")  # Convert the image to np.array
            results = self.model.track(frame, persist=True,classes=0, half=True, device=0, imgsz=320)  # Compute predictions
            #out = results[0].plot()  # This plots the prediction on the original image
            # Publish after converting to ROS format
            boxes = results[0].boxes.xyxy.cpu().numpy()
            
            self.publish_numpy_array(boxes)
            #self.detector_publisher.publish(self.bridge.cv2_to_imgmsg(out, "rgb8"))

        except CvBridgeError as e:
            print(e)

    def publish_numpy_array(self, numpy_array):
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        if numpy_array.size != 0:
            dim.label= 'full'
            dim.stride = 4
            dim.size = numpy_array.shape[0]
            msg.layout.dim.append(dim)
            msg.data = numpy_array.flatten().tolist()
        else:
            dim.label= 'empty'
            dim.stride = 0
            dim.size = 0
            msg.layout.dim.append(dim)
            msg.data = [0.0]

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    detector= DetectorAndTracker()
    print("Detecting and tracking...")
    rclpy.spin(detector)
    print("qualsiasi cosa")
    
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
