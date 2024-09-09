# service client: send request, get response
import random

from msg_srv_action_interface_example.src import ArithmeticOperator
import rclpy
from rclpy.node import Node

class Operator(Node):
    def __init__(self):
        super().__init__('operator')
        
        self.arithmetic_service_client = self.create_client(
            ArithmeticOperator,
            'arithmetic_operator'
        )
        
        # check whether service server is executing every 0.1 second
        while not self.arithmetic_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('arithmetic_operator service is not available')
        
        def send_request(self): # send request, receive response
            service_request = ArithmeticOperator.Request()
            service_request.arithmetic_operator = random.randint(1, 4)
            futures = self.arithmetic_service_client.call_async(service_request)
            return futures # response and service status

def main(args=None):
    rclpy.init(args=args)
    operator = Operator()
    future = operator.send_request() # send req, receive res
    user_trigger = True # ensures very first iteration to occur
    try:
        while rclpy.ok():
            if user_trigger is True: 
                rclpy.spin_once(operator) # single ieteration spinning
                if future.done():
                    try:
                        service_response = future.result()
                    except Exception as e:  # noqa: B902
                        operator.get_logger().warn('Service call failed: {}'.format(str(e)))
                    else:
                        operator.get_logger().info(
                            'Result: {}'.format(service_response.arithmetic_result))
                        user_trigger = False 
            else: # from second iterations
                input('Press Enter for next service call.')
                future = operator.send_request() # send request again
                user_trigger = True

    except KeyboardInterrupt:
        operator.get_logger().info('Keyboard Interrupt (SIGINT)')

    operator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()