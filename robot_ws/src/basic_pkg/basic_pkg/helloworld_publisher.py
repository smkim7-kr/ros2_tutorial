import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class HelloworldPublisher(Node):
    def __init__(self):
        super().__init__('helloworld_publisher')
        qos_profile = QoSProfile(depth=10) # buffer max 10 data in case of communication failure
        self.helloworld_publisher = self.create_publisher(String, 'helloworld', qos_profile) # topic name is /helloworld
        self.timer = self.create_timer(1, self.publish_helloworld_msg) ## execute callback function every 1 second
        self.count = 0 
    
    def publish_helloworld_msg(self): # callback function
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.helloworld_publisher.publish(msg) # publish msg
        self.get_logger().info(f'Published message: {msg.data}') # similar functionality to print function
        self.count += 1
    
def main(args=None):
    rclpy.init(args=args)
    node = HelloworldPublisher()
    
    try:
        rclpy.spin(node) # execute callback function repeated times
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown() # finish node

if __name__ == '__main__':
    main()

        
    