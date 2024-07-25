import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Handler(Node):
    def __init__(self):
        super().__init__("simple_nav_node") #Init node
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10) #Publisher
        self.create_timer(1, self.timer_callback) #Exec callback each 1 second
        
    def timer_callback(self):
        msg = Twist() # Create message
        msg.linear.x = 0.31 # Set linear speed to 0.31 m/s on x axis
        self.get_logger().info(f"Moving at {msg.linear.x} m/s")
        self.pub.publish(msg)

def main():
    rclpy.init()
    handller = Handler()
    rclpy.spin(handller)
    handller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()