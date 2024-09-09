# ros2 run logger logging
# ros2 run logger logging --ros-args --log-level debug

import rclpy
import random
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import String

class Logger(Node):
    def __init__(self):
        super().__init__("logging_node")
        self.publisher = self.create_publisher(String, 'logging_demo', 10)
        self.timer = self.create_timer(1.0, self.increment_until_best)
        self.count = 0
        self.max_val = 10
    
    def increment_until_best(self):
        self.get_logger().log(
            'Start logging node',
            LoggingSeverity.INFO,
            once=True # will only log this once
        )
        
        msg = String()
        self.val = random.randint(0, self.max_val)
        msg.data = f'current value: {self.val}' # topic is string type
        
        if self.exclude_zero():
            self.get_logger().warn("Excluded 0!")
        else:
            self.publisher.publish(msg)
            self.get_logger().info(f'Published {msg.data}')
        
            if self.val == self.max_val:
                self.get_logger().info(f'Best value picked, cumulative count {self.count}')
                self.count = 0
            elif self.val % 2 == 0: 
                self.get_logger().debug(f'Current value {self.val} is even')
                self.count += 1
            else:
                self.get_logger().debug(f'Current value {self.val} is odd')
                self.count += 1
        
    def exclude_zero(self):
        if self.val == 0:
            self.get_logger().error('0 is not welcomed!')
            return True
        else:
            return False
        
def main(args=None):
    rclpy.init(args=args)
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()