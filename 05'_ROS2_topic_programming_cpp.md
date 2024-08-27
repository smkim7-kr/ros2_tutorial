
# ROS 2 Topic Programming

## 1. Introduction to Topics

A Topic in ROS 2 is a method for asynchronous, one-way communication where messages are sent by publishers and received by subscribers. It can handle 1:1 communication as well as 1:N, N:1, and N:N communications. Topics are the most widely used communication method in ROS.

In this tutorial, we will create a topic that publishes the current time (POSIX Time) and two variables, `a` and `b`, for arithmetic operations, as shown in Figure 1. For a detailed conceptual understanding of topics, refer to the "ROS 2 Topics" [lecture](link).

## 2. Topic Publisher Code

The source code for the topic publisher (the `argument` node) is available in the [GitHub repository](link). The relevant code files are:

- `topic_service_action_rclcpp_example/include/arithmetic/argument.hpp`
- `topic_service_action_rclcpp_example/src/arithmetic/argument.cpp`

### 2.1 Header File: `argument.hpp`

The `argument.hpp` file includes essential libraries for ROS 2, such as `rclcpp` and a custom message type `ArithmeticArgument`. The `Argument` class inherits from `rclcpp::Node` and contains variables for random number generation, a publisher, a timer, and a parameter client.

```cpp
#ifndef ARITHMETIC__ARGUMENT_HPP_
#define ARITHMETIC__ARGUMENT_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "msg_srv_action_interface_example/msg/arithmetic_argument.hpp"

class Argument : public rclcpp::Node
{
public:
  using ArithmeticArgument = msg_srv_action_interface_example::msg::ArithmeticArgument;

  explicit Argument(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Argument();

private:
  void publish_random_arithmetic_arguments();
  void update_parameter();

  float min_random_num_;
  float max_random_num_;

  rclcpp::Publisher<ArithmeticArgument>::SharedPtr arithmetic_argument_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
};
#endif  // ARITHMETIC__ARGUMENT_HPP_
```

### 2.2 Source File: `argument.cpp`

The `argument.cpp` file implements the `Argument` class. It sets up the node, initializes parameters, and creates a publisher with a defined QoS profile. The publisher sends random values for `a` and `b` every second, which are logged and sent as messages.

```cpp
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "arithmetic/argument.hpp"

using namespace std::chrono_literals;

Argument::Argument(const rclcpp::NodeOptions & node_options)
: Node("argument", node_options),
  min_random_num_(0.0),
  max_random_num_(0.0)
{
  // Parameter initialization
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
  this->declare_parameter("min_random_num", 0.0);
  min_random_num_ = this->get_parameter("min_random_num").get_value<float>();
  this->declare_parameter("max_random_num", 9.0);
  max_random_num_ = this->get_parameter("max_random_num").get_value<float>();
  this->update_parameter();

  // QoS setup and publisher creation
  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  arithmetic_argument_publisher_ =
    this->create_publisher<ArithmeticArgument>("arithmetic_argument", QOS_RKL10V);

  // Timer setup to publish messages every second
  timer_ =
    this->create_wall_timer(1s, std::bind(&Argument::publish_random_arithmetic_arguments, this));
}

void Argument::publish_random_arithmetic_arguments()
{
  // Random number generation
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> distribution(min_random_num_, max_random_num_);

  // Create and publish message
  msg_srv_action_interface_example::msg::ArithmeticArgument msg;
  msg.stamp = this->now();
  msg.argument_a = distribution(gen);
  msg.argument_b = distribution(gen);
  arithmetic_argument_publisher_->publish(msg);

  // Log published values
  RCLCPP_INFO(this->get_logger(), "Published argument_a %.2f", msg.argument_a);
  RCLCPP_INFO(this->get_logger(), "Published argument_b %.2f", msg.argument_b);
}

void Argument::update_parameter()
{
  // Parameter update implementation
}

void print_help()
{
  printf("For argument node:
");
  printf("node_name [-h]
");
  printf("Options:
");
  printf("	-h Help           : Print this help function.
");
}

int main(int argc, char * argv[])
{
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);

  auto argument = std::make_shared<Argument>();

  rclcpp::spin(argument);

  rclcpp::shutdown();

  return 0;
}
```

## 3. Topic Subscriber Code

The topic subscriber (`calculator` node) code is also available in the [GitHub repository](link). The relevant files are:

- `topic_service_action_rclcpp_example/include/calculator/calculator.hpp`
- `topic_service_action_rclcpp_example/src/calculator/calculator.cpp`

### 3.1 Subscriber Setup

The `calculator` node subscribes to the `arithmetic_argument` topic. The subscriber is initialized with a QoS profile similar to the publisher. It uses a lambda function for the callback, which processes the received messages.

```cpp
const auto QOS_RKL10V =
  rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

arithmetic_argument_subscriber_ = this->create_subscription<ArithmeticArgument>(
  "arithmetic_argument",
  QOS_RKL10V,
  [this](const ArithmeticArgument::SharedPtr msg) -> void
  {
    argument_a_ = msg->argument_a;
    argument_b_ = msg->argument_b;

    RCLCPP_INFO(
      this->get_logger(),
      "Subscribed at: sec %ld nanosec %ld",
      msg->stamp.sec,
      msg->stamp.nanosec);

    RCLCPP_INFO(this->get_logger(), "Subscribed argument a : %.2f", argument_a_);
    RCLCPP_INFO(this->get_logger(), "Subscribed argument b : %.2f", argument_b_);
  }
);
```

## 4. Review of Publisher and Subscriber

To summarize the steps for creating a topic publisher and subscriber:

### 4.1 Topic Publisher (Sending Data)
1. Set up the Node.
2. Configure QoS settings.
3. Create a publisher with `create_publisher`.
4. Implement the publishing function.

### 4.2 Topic Subscriber (Receiving Data)
1. Set up the Node.
2. Configure QoS settings.
3. Create a subscriber with `create_subscription`.
4. Implement the subscription function.

## 5. Running the Nodes

To run the nodes, use the following commands:

```bash
$ ros2 run topic_service_action_rclcpp_example calculator
$ ros2 run topic_service_action_rclcpp_example argument
```

Ensure the nodes are correctly set up in the CMakeLists.txt file with `add_executable`.

## 6. Conclusion

In this tutorial, we explored ROS 2 topic programming. The next tutorials will cover services, actions, parameters, and launch files. The source code used in this tutorial is available in the [GitHub repository](link).
