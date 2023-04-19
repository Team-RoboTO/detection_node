import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/zedm/zed_node/left/image_rect_color',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('Image', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

    # rilascia le risorse di OpenCV
    cv2.destroyAllWindows()

    # cancella il context di ROS2
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
