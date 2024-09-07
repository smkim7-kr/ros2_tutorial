
# ROS 2 Action Programming

## 1. Action

An action in ROS 2 is a method of asynchronous and synchronous two-way communication. It involves an Action Client that specifies an action goal and an Action Server that performs a task based on this goal. While performing the task, the server sends intermediate results as feedback and final results as the outcome.

In this tutorial, we will create an Action Client that specifies an action goal and an Action Server that performs the task, sending feedback and results. For more details on actions, refer to the '011 ROS 2 Action' tutorial.

## 2. Action Server Code

The source code for the Action Server in the calculator node can be found in the GitHub repository:

- `topic_service_action_rclcpp_example/include/calculator/calculator.hpp`
- `topic_service_action_rclcpp_example/src/calculator/calculator.cpp`

The Calculator node includes a Topic Subscriber, a Service Server, and an Action Server. We will focus on the Action Server part.

### Key Components

- **arithmetic_action_server**: This is an `rclcpp_action::Server` smart pointer. It is initialized using the `create_server` function, which takes the node interfaces, action name, and callback functions as arguments.

```cpp
arithmetic_action_server_ = rclcpp_action::create_server<ArithmeticChecker>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "arithmetic_checker",
    std::bind(&Calculator::handle_goal, this, _1, _2),
    std::bind(&Calculator::handle_cancel, this, _1),
    std::bind(&Calculator::execute_checker, this, _1)
);
```

### Callback Functions

- **handle_goal**: Called when the Action Client requests a goal. It checks the goal and decides whether to proceed.

```cpp
rclcpp_action::GoalResponse Calculator::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ArithmeticChecker::Goal> goal)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
```

- **handle_cancel**: Called when the Action Client requests to cancel a goal. It handles the cancellation and any ongoing tasks.

```cpp
rclcpp_action::CancelResponse Calculator::handle_cancel(
  const std::shared_ptr<GoalHandleArithmeticChecker> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}
```

- **execute_checker**: The main function that performs the task based on the goal. It provides feedback and sets the result.

```cpp
void Calculator::execute_checker(const std::shared_ptr<GoalHandleArithmeticChecker> goal_handle)
{
  auto feedback_msg = std::make_shared<ArithmeticChecker::Feedback>();
  float total_sum = 0.0;
  float goal_sum = goal_handle->get_goal()->goal_sum;

  while ((total_sum < goal_sum) && rclcpp::ok()) {
    total_sum += argument_result_;
    feedback_msg->formula.push_back(argument_formula_);
    goal_handle->publish_feedback(feedback_msg);
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    auto result = std::make_shared<ArithmeticChecker::Result>();
    result->all_formula = feedback_msg->formula;
    result->total_sum = total_sum;
    goal_handle->succeed(result);
  }
}
```

## 3. Action Client Code

The source code for the Action Client in the checker node can be found in the GitHub repository:

- `topic_service_action_rclcpp_example/include/checker/checker.hpp`
- `topic_service_action_rclcpp_example/src/checker/checker.cpp`

### Key Components

- **Checker Class**: Inherits from `rclcpp::Node`. It initializes the Action Client and sends the action goal.

```cpp
class Checker : public rclcpp::Node
{
public:
  explicit Checker(float goal_sum, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Checker();

private:
  void send_goal_total_sum(float goal_sum);
  void get_arithmetic_action_goal(std::shared_future<rclcpp_action::ClientGoalHandle<ArithmeticChecker>::SharedPtr> future);
  void get_arithmetic_action_feedback(GoalHandleArithmeticChecker::SharedPtr, const std::shared_ptr<const ArithmeticChecker::Feedback> feedback);
  void get_arithmetic_action_result(const GoalHandleArithmeticChecker::WrappedResult & result);

  rclcpp_action::Client<ArithmeticChecker>::SharedPtr arithmetic_action_client_;
};
```

- **send_goal_total_sum**: Sends the action goal to the server.

```cpp
void Checker::send_goal_total_sum(float goal_sum)
{
  if (!this->arithmetic_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    return;
  }

  auto goal_msg = ArithmeticChecker::Goal();
  goal_msg.goal_sum = goal_sum;

  auto send_goal_options = rclcpp_action::Client<ArithmeticChecker>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&Checker::get_arithmetic_action_goal, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&Checker::get_arithmetic_action_feedback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&Checker::get_arithmetic_action_result, this, _1);
  this->arithmetic_action_client_->async_send_goal(goal_msg, send_goal_options);
}
```

### Callback Functions

- **get_arithmetic_action_goal**: Handles the server's response to the goal.

```cpp
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
```

- **get_arithmetic_action_feedback**: Processes feedback from the server during goal execution.

```cpp
void Checker::get_arithmetic_action_feedback(
  GoalHandleArithmeticChecker::SharedPtr,
  const std::shared_ptr<const ArithmeticChecker::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Action feedback: ");
  for (const auto & formula : feedback->formula) {
    RCLCPP_INFO(this->get_logger(), "	%s ", formula.c_str());
  }
}
```

- **get_arithmetic_action_result**: Handles the final result from the server.

```cpp
void Checker::get_arithmetic_action_result(
  const GoalHandleArithmeticChecker::WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Action succeeded!");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Action failed.");
  }
  rclcpp::shutdown();
}
```

## 4. Summary

To recap, setting up an action involves:

### Action Server
1. Node setup
2. `create_server` setup
3. Goal, cancel, and execute callback functions setup

### Action Client
1. Node setup
2. `create_client` setup
3. Goal response, feedback, and result callback functions setup

## 5. Running the Nodes

To run the nodes, use the following commands:

```bash
$ ros2 run topic_service_action_rclcpp_example calculator 
$ ros2 run topic_service_action_rclcpp_example checker
```

These nodes are configured in the `CMakeLists.txt` file. The `add_executable` tag is used to make the source files executable.

## 6. Conclusion

This tutorial covered ROS 2 Action Programming. The next tutorial will explore parameters, launch files, and more. The source code is available in the GitHub repository:

- [topic_service_action_rclcpp_example](https://github.com/robotpilot/ros2-seminar-examples)
