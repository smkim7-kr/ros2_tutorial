# action client: send goal, get feedback and result
import rclpy
import argparse
import sys

from action_msgs.msg import GoalStatus
from msg_srv_action_interface_example.action import ArithmeticChecker
from rclpy.action import ActionClient
from rclpy.node import Node

class Checker(Node):
    def __init__(self):
        super().__init__('checker')
        self.arithmetic_action_client = ActionClient(
            self,
            ArithmeticChecker,
            'arithmetic_checker')
    
    def send_goal_total_sum(self, goal_sum):
        # send action goal to action server and retreive feedback and result with callback function
        wait_count = 1
        while not self.arithmetic_action_client.wait_for_server(timeout_sec=0.1):
            # attempt to connect to action server
            if wait_count > 3:
                self.get_logger().warning('Arithmetic action server isn\'t available')
                return False
            wait_count += 1
        goal_msg = ArithmeticChecker.Goal()
        goal_msg.goal_sum = (float)(goal_sum) # set action goal
        self.send_goal_future = self.arithmetic_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.get_arithmetic_action_feedback) # callback run when retreive feedback
        self.send_goal_future.add_done_callback(self.get_arithmetic_action_goal) # callback run when retreive result

    def get_arithmetic_action_feedback(self, feedback_msg): 
        # action feedback callback function
        action_feedback = feedback_msg.feedback.formula
        self.get_logger().info(f'Action feedback: {action_feedback}')
    
    def get_arithmetic_action_goal(self, future):
        # action result callback function
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Action goal rejected')
            return
        self.get_logger().info('Action goal accepted')
        self.action_result_future = goal_handle.get_result_async()
        # callback that outputs result
        self.action_result_future.add_done_callback(self.get_arithmetic_action_result)

    def get_arithmetic_action_result(self, future):
        action_status = future.result().status
        action_result = future.result().result
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Action succeeded!')
            self.get_logger().info(
                f'Action result(all formula): {action_result.all_formula}')
            self.get_logger().info(
                f'Action result(total sum): {action_result.total_sum}')
        else:
            self.get_logger().warning(
                f'Action failed with status: {action_status}')
            
def main(argv=sys.argv[1:]): # args=None ignores CLI argumetns
   # parsing CLI arguments, argv[0] is excluded since it is executed path
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter) # 1) Create parser
    # 2) add argument to parser
    parser.add_argument(
        '-g',
        '--goal_total_sum',
        type=int,
        default=50,
        help='Target goal value of total sum')
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    args = parser.parse_args() # 3) parse arugments

    rclpy.init(args=args.argv)
    try:
        checker = Checker()
        checker.send_goal_total_sum(args.goal_total_sum) # send action goal (before node running spin!)
        try:
            rclpy.spin(checker)
        except KeyboardInterrupt:
            checker.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            checker.arithmetic_action_client.destroy() # action node (both server/client) required to destroyed explicitly
            checker.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()