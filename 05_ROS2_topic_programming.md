
# ROS 2 Topic Programming

## 1. Topic Overview
A topic in ROS is an asynchronous, one-way communication method where a publisher sends messages of type `msg` to subscribers. This is primarily a 1:1 communication model, but it also supports 1:N, N:1, and N:N configurations, making it the most widely used communication method in ROS.

In this tutorial, we will create a topic publisher that publishes the current time and two variables, `a` and `b`, and a topic subscriber that receives this data. For a deeper understanding of topics, refer to the '009 ROS 2 Topic' tutorial.

![Topic Publisher and Subscriber](#)

## 2. Topic Publisher Code
The topic publisher, implemented in the `argument` node, is available in the following GitHub repository:

```plaintext
topic_service_action_rclpy_example/topic_service_action_rclpy_example/arithmetic/argument.py
```

### Full Source Code for the `argument` Node:
```python
import random
from msg_srv_action_interface_example.msg import ArithmeticArgument
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

class Argument(Node):
    def __init__(self):
        super().__init__('argument')
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        self.declare_parameter('min_random_num', 0)
        self.min_random_num = self.get_parameter('min_random_num').value
        self.declare_parameter('max_random_num', 9)
        self.max_random_num = self.get_parameter('max_random_num').value
        self.add_on_set_parameters_callback(self.update_parameter)

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.arithmetic_argument_publisher = self.create_publisher(
            ArithmeticArgument,
            'arithmetic_argument',
            QOS_RKL10V)

        self.timer = self.create_timer(1.0, self.publish_random_arithmetic_arguments)

    def publish_random_arithmetic_arguments(self):
        msg = ArithmeticArgument()
        msg.stamp = self.get_clock().now().to_msg()
        msg.argument_a = float(random.randint(self.min_random_num, self.max_random_num))
        msg.argument_b = float(random.randint(self.min_random_num, self.max_random_num))
        self.arithmetic_argument_publisher.publish(msg)
        self.get_logger().info('Published argument a: {0}'.format(msg.argument_a))
        self.get_logger().info('Published argument b: {0}'.format(msg.argument_b))

    def update_parameter(self, params):
        for param in params:
            if param.name == 'min_random_num' and param.type_ == Parameter.Type.INTEGER:
                self.min_random_num = param.value
            elif param.name == 'max_random_num' and param.type_ == Parameter.Type.INTEGER:
                self.max_random_num = param.value
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    try:
        argument = Argument()
        try:
            rclpy.spin(argument)
        except KeyboardInterrupt:
            argument.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            argument.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Components of the Publisher:
- **Node Initialization**: The `Argument` class extends `Node`, and the node is initialized with the name `argument`.
- **QoS Settings**: Using `QoSProfile`, the publisher is configured with reliable transmission, last-keep history, a depth of 10, and volatile durability.
- **Publisher Creation**: The publisher is created with `create_publisher`, which publishes messages of type `ArithmeticArgument` to the `arithmetic_argument` topic.
- **Timer and Publishing**: A timer is set to trigger the `publish_random_arithmetic_arguments` method every second, which publishes messages with randomly generated values for `a` and `b`.

## 3. Topic Subscriber Code
The topic subscriber, implemented in the `calculator` node, is available in the following GitHub repository:

```plaintext
topic_service_action_rclpy_example/topic_service_action_rclpy_example/calculator/calculator.py
```

### Subscriber Implementation:
- **Node Initialization**: The `Calculator` class extends `Node`, and the node is initialized with the name `calculator`.
- **QoS Settings**: The subscriber is configured similarly to the publisher with the same QoS settings.
- **Subscriber Creation**: The subscriber is created with `create_subscription`, subscribing to the `arithmetic_argument` topic and invoking the `get_arithmetic_argument` callback function upon receiving a message.

```python
class Calculator(Node):
    def __init__(self):
        super().__init__('calculator')
        self.argument_a = 0.0
        self.argument_b = 0.0
        self.callback_group = ReentrantCallbackGroup()

        QOS_RKL10V = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.arithmetic_argument_subscriber = self.create_subscription(
            ArithmeticArgument,
            'arithmetic_argument',
            self.get_arithmetic_argument,
            QOS_RKL10V,
            callback_group=self.callback_group)

    def get_arithmetic_argument(self, msg):
        self.argument_a = msg.argument_a
        self.argument_b = msg.argument_b
        self.get_logger().info('Subscribed at: {0}'.format(msg.stamp))
        self.get_logger().info('Subscribed argument a: {0}'.format(self.argument_a))
        self.get_logger().info('Subscribed argument b: {0}'.format(self.argument_b))
```

### Key Components of the Subscriber:
- **Callback Function**: The `get_arithmetic_argument` function is invoked when a message is received, logging the received values for `a` and `b`.

## 4. Topic Publisher and Subscriber Recap
To summarize, the steps to implement a topic in ROS 2 are as follows:

### For a Publisher:
1. **Node Setup**
2. **QoS Configuration**
3. **Publisher Creation**
4. **Publishing Function**

### For a Subscriber:
1. **Node Setup**
2. **QoS Configuration**
3. **Subscription Creation**
4. **Subscription Callback Function**

## 5. Node Execution Code
The execution of the nodes is configured in the `setup.py` file under the `entry_points` section, which links the node names to their respective executable scripts.

```python
entry_points={
    'console_scripts': [
        'argument = topic_service_action_rclpy_example.arithmetic.argument:main',
        'operator = topic_service_action_rclpy_example.arithmetic.operator:main',
        'calculator = topic_service_action_rclpy_example.calculator.main:main',
        'checker = topic_service_action_rclpy_example.checker.main:main',
    ],
},
```

### Running the Nodes:
- **Argument Node**: Initializes the node, creates an instance of `Argument`, and spins the node.
- **Calculator Node**: Initializes the node, creates an instance of `Calculator`, and uses `MultiThreadedExecutor` to handle callbacks concurrently.

## 6. Conclusion
This tutorial covered the basics of ROS 2 topic programming. In the upcoming tutorials, we will analyze services, actions, parameters, and launch files. The source code used in this tutorial is available in the [GitHub repository](https://github.com/robotpilot/ros2-seminar-examples). For additional details, refer to the RCLPY API documentation and the official ROS examples.

