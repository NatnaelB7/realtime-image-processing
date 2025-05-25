import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class WebcamDriver(Node):
    def __init__(self):
        super().__init__('webcam_driver')
        self.publisher_ = self.create_publisher(Image, '/webcam/image_raw', 10)
        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        timestamp = str(int(time.time()))
        cv2.putText(frame, f"Time: {timestamp}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
        img_msg = self.br.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WebcamDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

