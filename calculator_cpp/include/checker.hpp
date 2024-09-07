#ifndef CHECKER__CHECKER_HPP_
#define CHECKER__CHECKER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "msg_srv_action_interface_example/action/arithmetic_checker.hpp"


class Checker : public rclcpp::Node // inherit Node class
{
public:
  using ArithmeticChecker = msg_srv_action_interface_example::action::ArithmeticChecker;
  using GoalHandleArithmeticChecker = rclcpp_action::ClientGoalHandle<ArithmeticChecker>;

  // constructor
  explicit Checker(
    float goal_sum, // goal sum is parameter passed from cli arguments
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Checker();

private:
// request action to action client
  void send_goal_total_sum(float goal_sum);
  
  // callback functions
  void get_arithmetic_action_goal(
    std::shared_future<rclcpp_action::ClientGoalHandle<ArithmeticChecker>::SharedPtr> future);

  void get_arithmetic_action_feedback(
    GoalHandleArithmeticChecker::SharedPtr,
    const std::shared_ptr<const ArithmeticChecker::Feedback> feedback);

  void get_arithmetic_action_result(
    const GoalHandleArithmeticChecker::WrappedResult & result);
  
  // smart pointer for action client
  rclcpp_action::Client<ArithmeticChecker>::SharedPtr arithmetic_action_client_;
};
#endif  // CHECKER__CHECKER_HPP_