import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, '/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Capture image every 100ms
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Open the default camera (CSI camera on Raspberry Pi)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            return

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert the frame to a ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)

    node.cap.release()  # Release the camera
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
