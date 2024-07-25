import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    ''' This class is used to publish the images from the camera to the ROS2 network. '''
    
    def __init__(self):
        ''' The constructor of the CameraNode class. '''
        super().__init__('camera_node')
        # Publisher to publish the images from the camera.
        self.publisher_ = self.create_publisher(Image, '/camera_image', 10)
        # Timer to publish the images from the camera.
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        # Bridge between OpenCV and ROS2.
        self.bridge = CvBridge()
        # Camera object.
        self.cap = cv2.VideoCapture(0)

    def timer_callback(self) -> None:
        ''' This method is used to publish the images from the camera to the ROS2 network. '''
        ret, frame = self.cap.read()
        if ret:
            # Publish the images from the camera.
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(msg)

def main(args=None):
    # Initialize the ROS2 node.
    rclpy.init(args=args)
    # Create an instance of the CameraNode class.
    camera_node = CameraNode()
    # Spin the node.
    rclpy.spin(camera_node)
    # Destroy the node.
    camera_node.destroy_node()
    # Shutdown the ROS2 node.
    rclpy.shutdown()

if __name__ == '__main__':
    # Run the main function.
    main()