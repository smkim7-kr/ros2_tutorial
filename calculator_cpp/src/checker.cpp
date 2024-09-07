#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "checker.hpp"

// Checker constructor
Checker::Checker(float goal_sum, const rclcpp::NodeOptions & node_options)
: Node("checker", node_options)
{
  arithmetic_action_client_ = rclcpp_action::create_client<ArithmeticChecker>( // instantiate action cllient
    this->get_node_base_interface(), // parameters with interfaces
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "arithmetic_checker");

  send_goal_total_sum(goal_sum); // send action goal to action server in the constructor
}

Checker::~Checker()
{
}

// send action goal
void Checker::send_goal_total_sum(float goal_sum) // goal sum is parameter passed from cli arguments
{
    if (!this->arithmetic_action_client_) {
        RCLCPP_WARN(this->get_logger(), "Action client not initialized");
    }

    if (!this->arithmetic_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_WARN(this->get_logger(), "Arithmetic action server is not available.");
        return;
    }

    auto goal_msg = ArithmeticChecker::Goal();
    goal_msg.goal_sum = goal_sum;

    // Define the options and the lambda callbacks
    auto send_goal_options = rclcpp_action::Client<ArithmeticChecker>::SendGoalOptions();

    send_goal_options.goal_response_callback = 
        [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<ArithmeticChecker>> goal_handle) {
            this->get_arithmetic_action_goal(goal_handle);
        };

    send_goal_options.feedback_callback = 
        [this](rclcpp_action::ClientGoalHandle<ArithmeticChecker>::SharedPtr goal_handle,
              const std::shared_ptr<const ArithmeticChecker::Feedback> feedback) {
            this->get_arithmetic_action_feedback(goal_handle, feedback);
        };


    send_goal_options.result_callback = 
        [this](const rclcpp_action::ClientGoalHandle<ArithmeticChecker>::WrappedResult & result) {
            this->get_arithmetic_action_result(result);
        };

    this->arithmetic_action_client_->async_send_goal(goal_msg, send_goal_options);
}

// work like service communication: send actionn goal, retreive response
void Checker::get_arithmetic_action_goal(
  std::shared_future<GoalHandleArithmeticChecker::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "Action goal rejected.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Action goal accepted.");
  }
}

// callback that checks action feedback
void Checker::get_arithmetic_action_feedback(
  GoalHandleArithmeticChecker::SharedPtr,
  const std::shared_ptr<const ArithmeticChecker::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Action feedback: ");
  for (const auto & formula : feedback->formula) {
    RCLCPP_INFO(this->get_logger(), "\t%s ", formula.c_str());
  }
}

// callback that retreives actionn result, can conduct several client logic based on results
void Checker::get_arithmetic_action_result(
  const GoalHandleArithmeticChecker::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED: // goal success
      RCLCPP_INFO(this->get_logger(), "Action succeeded!");
      RCLCPP_INFO(this->get_logger(), "Action result(all formula): ");
      for (const auto & formula : result.result->all_formula) {
        RCLCPP_INFO(this->get_logger(), "\t%s ", formula.c_str());
      }
      RCLCPP_INFO(this->get_logger(), "Action result(total sum): ");
      RCLCPP_INFO(this->get_logger(), "\t%.2f ", result.result->total_sum);
      break;
    case rclcpp_action::ResultCode::ABORTED: // goal abort
      RCLCPP_WARN(this->get_logger(), "The action was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED: // goal cancel
      RCLCPP_WARN(this->get_logger(), "The action was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  rclcpp::shutdown();
}

void print_help()
{
  printf("For Node node:\n");
  printf("node_name [-h]\n");
  printf("Options:\n");
  printf("\t-h Help           : Print this help function.\n");
}

int main(int argc, char * argv[])
{
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);

  float goal_total_sum = 50.0;
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-g");
  if (nullptr != cli_option) {
    goal_total_sum = std::stof(cli_option);
  }
  printf("goal_total_sum : %2.f\n", goal_total_sum);

  auto checker = std::make_shared<Checker>(goal_total_sum);

  rclcpp::spin(checker);

  rclcpp::shutdown();

  return 0;
}