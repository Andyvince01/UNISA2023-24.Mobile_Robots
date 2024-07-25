import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class QRInput(Node):
    '''
    This class is a ROS2 node that publishes user input to a topic.
    It is used to simulate a QR code scanner when Turtlebot4 camera is not available.
    '''
    
    def __init__(self):
        ''' Initializes the QRInput node. '''
        super().__init__('user_input_node')
        self.publisher_ = self.create_publisher(String, '/cmd_qr', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('QRInput Node has been started.')

    def timer_callback(self):
        ''' Publishes user input to the topic. '''
        msg = String()
        msg.data = input("Enter your input: ")
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    ''' Main function to run the QRInput node.

    Parameters
    ----------
    args : _type_, optional
        Description of parameter 'args'. The default is None.
    '''
    # Initialize the ROS2 node.
    rclpy.init(args=args)
    # Create an instance of the QRInput class.
    qr_input_node = QRInput()
    # Spin the node.
    rclpy.spin(qr_input_node)
    # Destroy the node.
    qr_input_node.destroy_node()
    # Shutdown the ROS2 node.
    rclpy.shutdown()

if __name__ == '__main__':
    # Run the main function.
    main()