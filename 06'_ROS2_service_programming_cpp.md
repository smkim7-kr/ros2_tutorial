
# ROS 2 Service Programming Summary

## 1. Service

A service is a synchronous, bidirectional communication mechanism in ROS. It involves a client-server model where the client sends a request, and the server processes this request and sends back a response. The communication is usually one-to-one or many-to-one.

In this tutorial, we will create a service client that sends a request with an operator (e.g., +, -, *, /) and a service server that calculates the result using the existing variables `a` and `b` with the given operator, then sends back the result.

![Service Server and Client](image_link_here)

## 2. Service Server Code

The code for the service server node, `calculator`, can be found in the following GitHub repository:

- `topic_service_action_rclcpp_example/include/calculator/calculator.hpp`
- `topic_service_action_rclcpp_example/src/calculator/calculator.cpp`

The `Calculator` node includes a service server, among other functionalities. Here's an overview of the service server part:

The `Calculator` class inherits from `rclcpp::Node`. In the constructor, the `arithmetic_argument_server` is initialized using `create_service`, which takes the service name and a callback function (`get_arithmetic_operator`).

```cpp
auto get_arithmetic_operator =
  [this](
  const std::shared_ptr<ArithmeticOperator::Request> request,
  std::shared_ptr<ArithmeticOperator::Response> response) -> void
  {
    argument_operator_ = request->arithmetic_operator;
    argument_result_ =
      this->calculate_given_formula(argument_a_, argument_b_, argument_operator_);
    response->arithmetic_result = argument_result_;

    std::ostringstream oss;
    oss << std::to_string(argument_a_) << ' ' <<
      operator_[argument_operator_ - 1] << ' ' <<
      std::to_string(argument_b_) << " = " <<
      argument_result_ << std::endl;
    argument_formula_ = oss.str();

    RCLCPP_INFO(this->get_logger(), "%s", argument_formula_.c_str());
  };

arithmetic_argument_server_ =
  create_service<ArithmeticOperator>("arithmetic_operator", get_arithmetic_operator);
```

The `calculate_given_formula` function processes the request and returns the result:

```cpp
float Calculator::calculate_given_formula(
  const float & a,
  const float & b,
  const int8_t & operators)
{
  float argument_result = 0.0;
  ArithmeticOperator::Request arithmetic_operator;

  if (operators == arithmetic_operator.PLUS) {
    argument_result = a + b;
  } else if (operators == arithmetic_operator.MINUS) {
    argument_result = a - b;
  } else if (operators == arithmetic_operator.MULTIPLY) {
    argument_result = a * b;
  } else if (operators == arithmetic_operator.DIVISION) {
    argument_result = a / b;
    if (b == 0.0) {
      RCLCPP_ERROR(this->get_logger(), "ZeroDivisionError!");
      argument_result = 0.0;
      return argument_result;
    }
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Please make sure arithmetic operator(plus, minus, multiply, division).");
    argument_result = 0.0;
  }

  return argument_result;
}
```

## 3. Service Client Code

The code for the service client node, `operator`, can be found in the following GitHub repository:

- `topic_service_action_rclcpp_example/include/arithmetic/operator.hpp`
- `topic_service_action_rclcpp_example/src/arithmetic/operator.cpp`

The `Operator` node sends a service request and waits for a response. Here's an overview of the service client part:

```cpp
class Operator : public rclcpp::Node
{
public:
  using ArithmeticOperator = msg_srv_action_interface_example::srv::ArithmeticOperator;

  explicit Operator(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Operator();

  void send_request();

private:
  rclcpp::Client<ArithmeticOperator>::SharedPtr arithmetic_service_client_;
};
```

In the constructor, the `arithmetic_service_client_` is initialized using `create_client`. The `send_request` function is responsible for sending the service request:

```cpp
void Operator::send_request()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> distribution(1, 4);

  auto request = std::make_shared<ArithmeticOperator::Request>();
  request->arithmetic_operator = distribution(gen);

  using ServiceResponseFuture = rclcpp::Client<ArithmeticOperator>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Result %.2f", response->arithmetic_result);
      return;
    };

  auto future_result =
    arithmetic_service_client_->async_send_request(request, response_received_callback);
}
```

## 4. Recap: Service Server and Client

### Service Server (Responding to Requests)

1. Set up the node.
2. Configure the service server using `create_service`.
3. Implement the callback function to handle requests.

### Service Client (Sending Requests)

1. Set up the node.
2. Configure the service client using `create_client`.
3. Implement the request function to send service requests.

## 5. Node Execution

The nodes can be executed using the following commands:

```sh
$ ros2 run topic_service_action_rclcpp_example calculator
$ ros2 run topic_service_action_rclcpp_example operator
```

## 6. Conclusion

This tutorial covered ROS 2 service programming. The next tutorials will cover action, parameter, and launch programming. The code used in this tutorial is available on the following GitHub repository:

[ROS 2 Seminar Examples](https://github.com/robotpilot/ros2-seminar-examples)
