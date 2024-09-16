
# ROS 2 Service Programming

## 1. Service Overview
A service in ROS is a synchronous, bidirectional communication method. The component requesting the service is known as the Service Client, while the component responding to the request is the Service Server. Essentially, a service involves communication between a client that makes a specific request and a server that performs the requested task and returns the result.

In this tutorial, we will create a service client that sends a request and a service server that responds to the request, as illustrated in Figure 1. The client will randomly choose an operator (e.g., +, -, *, /) and send it to the server. The server will then use the received operator to compute the result based on pre-stored variables `a` and `b` and return the result to the client.

### Nodes Involved:
1. **Operator Node**: Sends a request to the calculator node with the chosen operator using the service name `arithmetic_operator`.
2. **Calculator Node**: Subscribes to the variables `a` and `b`, performs the calculation using the operator received from the operator node, and returns the result.

![Service Server and Client](#)

## 2. Service Server Code
The service server is implemented in the `calculator` node, with the source code available in the following GitHub repository:

```plaintext
topic_service_action_rclpy_example/topic_service_action_rclpy_example/calculator/calculator.py
```

### Key Components of the Server:
- **Service Declaration**: The service server is declared using the `create_service` function, specifying the service type as `ArithmeticOperator`, the service name as `arithmetic_operator`, and the callback function as `get_arithmetic_operator`. The callback group is set to allow multi-threaded execution.

```python
self.arithmetic_service_server = self.create_service(
    ArithmeticOperator,
    'arithmetic_operator',
    self.get_arithmetic_operator,
    callback_group=self.callback_group)
```

- **Callback Function**: The `get_arithmetic_operator` function is executed when a service request is received. It processes the request using the pre-stored variables `a` and `b`, performs the calculation, and returns the result as the response.

```python
def get_arithmetic_operator(self, request, response):
    self.argument_operator = request.arithmetic_operator
    self.argument_result = self.calculate_given_formula(
        self.argument_a,
        self.argument_b,
        self.argument_operator)
    response.arithmetic_result = self.argument_result
    self.argument_formula = '{0} {1} {2} = {3}'.format(
            self.argument_a,
            self.operator[self.argument_operator-1],
            self.argument_b,
            self.argument_result)
    self.get_logger().info(self.argument_formula)
    return response
```

- **Calculation Function**: The `calculate_given_formula` function performs the arithmetic operation based on the operator received in the service request.

```python
def calculate_given_formula(self, a, b, operator):
    if operator == ArithmeticOperator.Request.PLUS:
        self.argument_result = a + b
    elif operator == ArithmeticOperator.Request.MINUS:
        self.argument_result = a - b
    elif operator == ArithmeticOperator.Request.MULTIPLY:
        self.argument_result = a * b
    elif operator == ArithmeticOperator.Request.DIVISION:
        try:
            self.argument_result = a / b
        except ZeroDivisionError:
            self.get_logger().error('ZeroDivisionError!')
            self.argument_result = 0.0
            return self.argument_result
    else:
        self.get_logger().error(
            'Please make sure arithmetic operator(plus, minus, multiply, division).')
        self.argument_result = 0.0
    return self.argument_result
```

## 3. Service Server Execution Code
The `calculator` node serves multiple roles, including a topic subscriber, service server, and action server. The execution code initializes the node, sets up a multi-threaded executor, and runs the node. For more details, refer to the '029 Topic Programming (Python)' tutorial.

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor
from topic_service_action_rclpy_example.calculator.calculator import Calculator

def main(args=None):
    rclpy.init(args=args)
    try:
        calculator = Calculator()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(calculator)
        try:
            executor.spin()
        except KeyboardInterrupt:
            calculator.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            executor.shutdown()
            calculator.arithmetic_action_server.destroy()
            calculator.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. Service Client Code
The service client is implemented in the `operator` node, with the source code available in the following GitHub repository:

```plaintext
topic_service_action_rclpy_example/topic_service_action_rclpy_example/arithmetic/operator.py
```

### Full Source Code for the `operator` Node:
```python
import random
from msg_srv_action_interface_example.srv import ArithmeticOperator
import rclpy
from rclpy.node import Node

class Operator(Node):
    def __init__(self):
        super().__init__('operator')

        self.arithmetic_service_client = self.create_client(
            ArithmeticOperator,
            'arithmetic_operator')

        while not self.arithmetic_service_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warning('The arithmetic_operator service not available.')

    def send_request(self):
        service_request = ArithmeticOperator.Request()
        service_request.arithmetic_operator = random.randint(1, 4)
        futures = self.arithmetic_service_client.call_async(service_request)
        return futures

def main(args=None):
    rclpy.init(args=args)
    operator = Operator()
    future = operator.send_request()
    user_trigger = True
    try:
        while rclpy.ok():
            if user_trigger is True:
                rclpy.spin_once(operator)
                if future.done():
                    try:
                        service_response = future.result()
                    except Exception as e:
                        operator.get_logger().warn('Service call failed: {}'.format(str(e)))
                    else:
                        operator.get_logger().info(
                            'Result: {}'.format(service_response.arithmetic_result))
                        user_trigger = False
            else:
                input('Press Enter for next service call.')
                future = operator.send_request()
                user_trigger = True

    except KeyboardInterrupt:
        operator.get_logger().info('Keyboard Interrupt (SIGINT)')

    operator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Components of the Client:
- **Client Declaration**: The service client is declared using the `create_client` function, specifying the service type as `ArithmeticOperator` and the service name as `arithmetic_operator`.

```python
self.arithmetic_service_client = self.create_client(
    ArithmeticOperator,
    'arithmetic_operator')

while not self.arithmetic_service_client.wait_for_service(timeout_sec=0.1):
    self.get_logger().warning('The arithmetic_operator service not available.')
```

- **Request Sending**: The `send_request` function sends a service request with a randomly selected operator to the service server.

```python
def send_request(self):
    service_request = ArithmeticOperator.Request()
    service_request.arithmetic_operator = random.randint(1, 4)
    futures = self.arithmetic_service_client.call_async(service_request)
    return futures
```

## 5. Service Client Node Execution Code
The `operator` node, part of the `topic_service_action_rclpy_example` package, is executed using the `ros2 run` command. The execution code initializes the node, sends a service request, and processes the response.

```python
def main(args=None):
    rclpy.init(args=args)
    operator = Operator()
    future = operator.send_request()
    user_trigger = True
    try:
        while rclpy.ok():
            if user_trigger is True:
                rclpy.spin_once(operator)
                if future.done():
                    try:
                        service_response = future.result()
                    except Exception as e:
                        operator.get_logger().warn('Service call failed: {}'.format(str(e)))
                    else:
                        operator.get_logger().info(
                            'Result: {}'.format(service_response.arithmetic_result))
                        user_trigger = False
            else:
                input('Press Enter for next service call.')
                future = operator.send_request()
                user_trigger = True

    except KeyboardInterrupt:
        operator.get_logger().info('Keyboard Interrupt (SIGINT)')

    operator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Additional Notes:
- **Continuous Execution**: Unlike topics, services are not continuously executed but are triggered by user input. After the initial request, the user can press Enter to send another request.

## 6. Conclusion
This tutorial covered the basics of ROS 2 service programming. In the upcoming tutorials, we will explore actions, parameters, and launch files. The source code used in this tutorial is available in the [GitHub repository](https://github.com/robotpilot/ros2-seminar-examples). For more details, refer to the RCLPY API documentation and the official ROS examples.

