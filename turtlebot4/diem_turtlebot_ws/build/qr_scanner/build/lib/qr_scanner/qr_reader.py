#!/usr/bin/python3
import cv2
import rclpy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.executors import MultiThreadedExecutor

class QRReader(Node):
    ''' QRReader class for ROS2
    
    This class is used to read the QR code from the images received from the camera.

    Attributes
    ----------
    _bridge : CvBridge
        The bridge between OpenCV and ROS2.
        
    _pub : String
        The publisher of the commands.
        
    _qr : cv2.QRCodeDetector
        The QR code detector.
        
    _sub : CompressedImage
        The subscriber of the images.
        
    Methods
    -------
    loop() -> None
        This method is used to read the QR code from the images received from the camera.
        
    Getters
    -------
    bridge() -> CvBridge
        Returns the bridge between OpenCV and ROS2.
        
    pub() -> String
        Returns the publisher of the commands.
        
    qr() -> cv2.QRCodeDetector
        Returns the QR code detector.
        
    sub() -> CompressedImage
        Returns the subscriber of the images.
        
    Setters
    -------
    bridge(bridge: CvBridge) -> None
        Sets the bridge between OpenCV and ROS2.
        
    pub(pub: String) -> None
        Sets the publisher of the commands.
    
    qr(qr: cv2.QRCodeDetector) -> None
        Sets the QR code detector.
        
    sub(sub: CompressedImage) -> None
        Sets the subscriber of the images.   
    '''
    __slots__ = ['_bridge', '_pub', '_qr', '_sub']

    # =============================================================================
    # Constructor
    # =============================================================================
    def __init__(self) -> None:
        ''' Constructor for the QRReader class. '''
        try:
            super().__init__('qr_reader')                           # Initialize the node
            self.get_logger().set_level(LoggingSeverity.DEBUG)      # Set the logging level of the node
            
            # ATTRIBUTES
            self._bridge = CvBridge()   
            self._qr = cv2.QRCodeDetector()
                        
            # PUBLISHER
            self._pub = self.create_publisher(msg_type=String, topic='/qr_reader', qos_profile=1)
            
            # SUBSCRIBER
            self._sub = self.create_subscription(msg_type=Image, topic='/oakd/rgb/preview/image_raw', callback=self._scan_images, qos_profile=1)
                                             
        except Exception as e:
            self.get_logger().error(f'Error in the QRReader\'s constructor: {e}')
    
    def loop(self) -> None:
        rclpy.spin(self)
    
    # =============================================================================
    # Private methods + Callbacks
    # =============================================================================
    def _scan_images(self, frame:Image) -> None:
        ''' This method is used to scan the images from the camera received from the topic. It detects the QR code and publishes the command.

        Parameters
        ----------
        frame : CompressedImage
            The frame grabbed from the camera.
        '''        

        try:
            image = self._bridge.imgmsg_to_cv2(frame, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Error in the Camera\'s scan images: {e}')
            
        try:
            command, points, _ = self._qr.detectAndDecode(image)
        except Exception as e:
            self.get_logger().error(f'Error in the Camera\'s detectAndDecode: {e}')
            command = None   
                
        if points is not None and points.any():
            # Draw the bounding box
            points = points[0].astype(int)
            n = len(points)
            for i in range(n):
                pt1 = tuple(points[i])
                pt2 = tuple(points[(i + 1) % n])
                cv2.line(image, pt1, pt2, color=(0, 255, 0), thickness=2)
            
        # Show the image
        cv2.imshow("QR Code", image)
        # Wait for a key press
        cv2.waitKey(1)
        
        if not command:
            return
        
        # Create the message 'String' and publish it        
        msg = String()
        msg.data = command.upper()
        # Publish the message
        self._pub.publish(msg)
        
    # =============================================================================
    # Getters
    # =============================================================================
    @property
    def bridge(self) -> CvBridge:
        return self._bridge
    
    @property
    def pub(self) -> String:
        return self._pub
    
    @property
    def qr(self) -> cv2.QRCodeDetector:
        return self._qr
    
    @property
    def sub(self) -> Image:
        return self._sub
    
    # =============================================================================
    # Setters
    # =============================================================================
    @bridge.setter
    def bridge(self, bridge: CvBridge) -> None:
        self._bridge = bridge
        
    @pub.setter
    def pub(self, pub: String) -> None:
        self._pub = pub
        
    @qr.setter
    def qr(self, qr: cv2.QRCodeDetector) -> None:
        self._qr = qr
        
    @sub.setter
    def sub(self, sub: Image) -> None:
        self._sub = sub    

def main():
    ''' Main function to start the QR reader node. '''
    # Initialize the ROS 2 node
    rclpy.init()
    # Create the executor
    executor = MultiThreadedExecutor()
    # Create the QR reader
    qr_reader = QRReader()
    # Add the QR reader to the executor
    executor.add_node(qr_reader)
    # Execute the loop - blocking call
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    # Destroy the node
    qr_reader.destroy_node()
    # Shutdown the ROS 2 client library
    rclpy.shutdown()

if __name__ == '__main__':
    # Run the main function
    main()