#!/usr/bin/python3
import cv2
import numpy as np
import rclpy
import threading
import queue

from cv_bridge import CvBridge, CvBridgeError
from qreader import QReader
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String

class QRScanner(Node):
    ''' QRScanner class for ROS2
    
    This class is used to read the QR code from the images received from the camera.
    '''
    __slots__ = ['_bridge', '_qr', '_timer', '_pub', '_sub', '_on_turtlebot', '_image_lock', '_image_queue']

    def __init__(self) -> None:
        ''' Constructor for the QRScanner class. '''
        try:
            super().__init__('qr_scanner')  # Initialize the node
            self.get_logger().set_level(LoggingSeverity.DEBUG)  # Set the logging level of the node

            # Get the simulation parameter from the launch file
            self.declare_parameter('on_turtlebot', False)
            self._on_turtlebot = self.get_parameter('on_turtlebot').get_parameter_value().bool_value
            self.get_logger().info("On Turtlebot? " + str(self._on_turtlebot))
            
            # ATTRIBUTES
            self._bridge = CvBridge()
            self._qr = QReader(model_size='n', min_confidence=0.35)
            self._timer = None

            # PUBLISHER
            self._pub = self.create_publisher(String, '/cmd_qr', 10)

            # SUBSCRIBER
            if self._on_turtlebot:
                self._sub = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self._scan_images, 10)
            else:
                self._sub = self.create_subscription(Image, '/camera_image', self._scan_images, 10)

            # Initialize the lock
            self._image_lock = threading.Lock()
            
            # Initialize the queue for images
            self._image_queue = queue.Queue()
            
        except Exception as e:
            self.get_logger().error(f'Error in the QRScanner\'s constructor: {e}')

    def loop(self) -> None:
        ''' This method is used to process images in the queue and display them using a lock to avoid conflicts. '''
        while rclpy.ok():
            # Get the image from the queue
            image = self._image_queue.get()

            # Display the image using the lock
            self._show_image(image)

            # Check for shutdown
            if not rclpy.ok():
                break

    def detect_and_decode_qr(self, frame):
        # Get the image that contains the QR code
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        command = None

        # Use the detect_and_decode function to get the decoded QR data
        # Utilizzare la funzione detect_and_decode per ottenere i dati decodificati del QR
        decoded_texts, detections = self._qr.detect_and_decode(image=image, return_detections=True)

        # Disegnare il poligono intorno ai QR code e annotare l'immagine
        for detection, decoded_text in zip(detections, decoded_texts):
            if detection:
                points = np.array(detection['polygon_xy'], dtype=np.int32)
                cv2.polylines(frame, [points], True, (0, 255, 0), 2)
                if decoded_text:
                    text = f"{decoded_text}"
                    cv2.putText(frame, text, (points[0][0], points[0][1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (60, 215, 60), 2)

        if decoded_texts:
            command = str(decoded_texts[0])
        
        return frame, command

    def _show_image(self, image: np.ndarray) -> None:
        ''' Display the image using a lock to avoid conflicts. '''
        with self._image_lock:
            cv2.imshow("@Diem Map", image)
            cv2.waitKey(1)

    def _scan_images(self, frame: Image) -> None:
        ''' This method is used to scan the images from the camera received from the topic. It detects the QR code and publishes the command. '''
        try:
            image = self._bridge.imgmsg_to_cv2(frame, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error in the Camera\'s scan images: {e}')
            return

        image, command = self.detect_and_decode_qr(image)

        # Put the image in the queue
        self._image_queue.put(image)

        if command is not None and command != '' and command != "None":

            self.get_logger().info(f'+++ [QR Code] {command}')

            # Create the message 'String' and publish it
            msg = String()
            msg.data = command.upper()
            self._pub.publish(msg)

def main():
    ''' Main function to start the QR reader node. '''
    # Initialize the ROS 2 node
    rclpy.init()
    # Create the executor
    executor = MultiThreadedExecutor()
    # Create the QR reader
    qr_scanner = QRScanner()
    # Add the QR reader to the executor
    executor.add_node(qr_scanner)
    # Execute the loop - blocking call
    try:
        # Run the loop in a separate thread
        loop_thread = threading.Thread(target=qr_scanner.loop)
        loop_thread.start()

        # Spin the node
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the loop thread
        qr_scanner._image_queue.put(None)  # Sentinel to stop the loop
        loop_thread.join()

        # Destroy the node
        qr_scanner.destroy_node()
        # Shutdown the ROS 2 client library
        rclpy.shutdown()

if __name__ == '__main__':
    # Run the main function
    main()