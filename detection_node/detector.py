#!/usr/bin/python3
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import time
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import Detection2DArray
import numpy as np
import pdb
import torch
import time
from time import sleep
from threading import Thread, Lock
import ros2_numpy as rnp



class DetectorAndTracker(Node):
    def __init__(self):
        super().__init__("DetectorAndTracker")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        self.bridge = CvBridge()
        self.rgb_subscriber = self.create_subscription(Image,
                                "/zed2/zed_node/rgb/image_rect_color",
                                self.run,qos_profile=qos_profile)
        self._pub = self.create_publisher(Detection2DArray, "/yolo/bboxs",
                                         qos_profile=qos_profile) 
          
          # This will automatically download the trained model
        print('cuda availability', torch.cuda.is_available())
        self.frame = None
        self.tt = -1.0
        self.model = YOLO("/home/auto/Downloads/best_armor.engine", task='detect')
        print("yolo is loaded...")

    def run(self, frame):
        #self.lock.acquire()
        
        self.frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        start = time.time()
        results = self.model.track(self.frame,classes=0, device=0,imgsz=(896,512), verbose=True,)  # Compute predictions
        t = time.time()
        print("model exec", t-start)
        if self.tt !=-1:
            delay = t - self.tt
        else:
            delay = -1.0
        self.tt = t
        print("delay", delay)
        detections_msg = Detection2DArray()
        detections_msg.header = frame.header
        r =results[0]
        if len(r.boxes) > 0:
            for b in r.boxes:
                box = b.xywh.cpu().numpy()
                detection = Detection2D()
                detection.bbox.center.position.x = float(box[0, 0])
                detection.bbox.center.position.y = float(box[0, 1])
                detection.bbox.size_x = float(box[0, 2])
                detection.bbox.size_y = float(box[0, 3]) 
                hypothesis = ObjectHypothesisWithPose()
                if b.is_track:     
                    hypothesis.hypothesis.class_id = str(b.id.cpu().numpy()[0])
                hypothesis.hypothesis.score = float(delay)
                detection.results.append(hypothesis)
                detections_msg.detections.append(detection)
            self._pub.publish(detections_msg)
            end=time.time()
            print("final elaboration", end-t)
            print("complete elaboration", end-start)



                    
        #self.lock.release()
def main(args=None):
    rclpy.init(args=args)
    print("start")
    detector= DetectorAndTracker()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()    

if __name__ == '__main__':
    main()
