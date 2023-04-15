#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import time
class DetectorAndTracker(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.bridge = CvBridge()
        self.rgb_subscriber = self.create_subscription(Image, "/zedm/zed_node/rgb/image_rect_color", self.run, 5)
        self.detector_publisher = self.create_publisher(Image, "/image/bbox",5)
        self.model = YOLO("yolov8n.pt")  # This will automatically download the trained model
        
    def run(self, frame):
        try:
            print("pre bridge")
            frame = self.bridge.imgmsg_to_cv2(frame, "rgb8")  # Convert the image to np.array
            print("pre track")
            results = self.model.track(frame, persist=True)  # Compute predictions
            print("pre esrtaction")
            out = results[0].plot()  # This plots the prediction on the original image
            # Publish after converting to ROS format
            print("pre publish")
            self.detector_publisher.publish(self.bridge.cv2_to_imgmsg(out, "rgb8"))
            print("after publish")
        except CvBridgeError as e:
            print(e)

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
