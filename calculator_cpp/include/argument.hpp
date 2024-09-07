#ifndef ARITHMETIC__ARGUMENT_HPP_
#define ARITHMETIC__ARGUMENT_HPP_

#include <chrono> 
#include <memory> // to use shared pointer
#include <string>
#include <utility> // to use different domains

#include "rclcpp/rclcpp.hpp"

#include "msg_srv_action_interface_example/msg/arithmetic_argument.hpp" // use own interface


class Argument : public rclcpp::Node // inherit Node class
{
public:
  using ArithmeticArgument = msg_srv_action_interface_example::msg::ArithmeticArgument;
  // constructor with NodeOptions parameter - can set options including onctect, IPC...
  explicit Argument(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()); 
  virtual ~Argument();

private:
  void publish_random_arithmetic_arguments();
  void update_parameter();
 
  // random value range
  float min_random_num_;
  float max_random_num_;

  rclcpp::Publisher<ArithmeticArgument>::SharedPtr arithmetic_argument_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
};
#endif  // ARITHMETIC__ARGUMENT_HPP_