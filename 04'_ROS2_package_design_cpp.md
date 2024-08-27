
# ROS Package Design

The difference between a robot program integrated with ROS and a general robot program is that processes are divided by purpose, and nodes are created for each process. You need to consider data communication between nodes when designing the program. In this tutorial, we'll introduce a single package designed to work with ROS 2 topics, services, and actions in C++. The example package consists of four nodes, as shown in Figure 1. Each node contains one or more topic publishers, topic subscribers, service...

![Figure 1: Package Configuration](#)

## Package Structure

This package is named `topic_service_action_rclcpp_example`, and each node, topic, service, and action has a unique name, as shown in Figure 2.

![Figure 2: Detailed Package Configuration](#)

The process described in Figure 2 works as follows:

- The `argument` node publishes the current time (POSIX time) and variables a and b with the topic name `arithmetic_argument`.
- The `calculator` node subscribes to the `arithmetic_argument` topic to receive the time and variables a and b.
- The `operator` node sends the operator (+,-,*,/) to the `calculator` node as a service request under the service name `arithmetic_operator`.
- The `calculator` node uses the operator and variables a and b to calculate the result and sends it back to the `operator` node as a service response.
- The `checker` node sends a goal to the `calculator` node with an action named `arithmetic_checker` to determine if the sum of the calculations exceeds a threshold.
- The `calculator` node accumulates the results of the calculations and sends feedback and the final result back to the `checker` node.

## Node Implementation

The `topic_service_action_rclcpp_example` package is composed of four nodes: `argument`, `operator`, `calculator`, and `checker`. The source code for these nodes can be found in the repository linked in the reference section. Detailed explanations of the nodes will be covered in upcoming tutorials.

## Package Configuration File (package.xml)

The `package.xml` file for the `topic_service_action_rclcpp_example` package is as follows:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>topic_service_action_rclcpp_example</name>
  <version>0.2.0</version>
  <description>ROS 2 rclcpp example package for the topic, service, action</description>
  <maintainer email="passionvirus@gmail.com">Pyo</maintainer>
  <license>Apache License 2.0</license>
  <author email="passionvirus@gmail.com">Pyo</author>
  <author email="routiful@gmail.com">Darby Lim</author>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>rclcpp_action</depend>
  <depend>msg_srv_action_interface_example</depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

The package format is the third version (used in ROS 2). The build tool is `ament_cmake`, and the dependencies include `rclcpp`, `rclcpp_action`, and the `msg_srv_action_interface_example` package.

## Build Configuration File (CMakeLists.txt)

The `CMakeLists.txt` for the `topic_service_action_rclcpp_example` package is as follows:

```cmake
cmake_minimum_required(VERSION 3.5)
project(topic_service_action_rclcpp_example)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(msg_srv_action_interface_example REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

add_executable(argument src/arithmetic/argument.cpp)
ament_target_dependencies(argument
  msg_srv_action_interface_example
  rclcpp
)

add_executable(calculator src/calculator/main.cpp src/calculator/calculator.cpp)
ament_target_dependencies(calculator
  msg_srv_action_interface_example
  rclcpp
  rclcpp_action
)

add_executable(checker src/checker/main.cpp src/checker/checker.cpp)
ament_target_dependencies(checker
  msg_srv_action_interface_example
  rclcpp
  rclcpp_action
)

add_executable(operator src/arithmetic/operator.cpp)
ament_target_dependencies(operator
  msg_srv_action_interface_example
  rclcpp
)

install(TARGETS
  argument
  calculator
  checker
  operator
  DESTINATION lib/\${PROJECT_NAME}
)

install(DIRECTORY launch param
  DESTINATION share/\${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

The `CMakeLists.txt` file handles the project configuration, dependencies, build instructions, and installation paths.

## Downloading and Building the Source Code

You can download and build the source code using the following commands:

```bash
cd ~/robot_ws/src
git clone https://github.com/robotpilot/ros2-seminar-examples.git
cd ~/robot_ws && colcon build --symlink-install
echo 'source ~/robot_ws/install/local_setup.bash' >> ~/.bashrc
source ~/.bashrc
```

Alternatively, you can use predefined aliases for convenience:

```bash
cw
cbp topic_service_action_rclcpp_example
```

## Running the Nodes

### Running the Calculator Node

First, run the calculator node, which acts as a topic subscriber, service server, and action server.

```bash
ros2 run topic_service_action_rclcpp_example calculator
```

### Running the Argument Node

In a new terminal, run the argument node, which acts as a topic publisher.

```bash
ros2 run topic_service_action_rclcpp_example argument
```

### Running the Operator Node

In another terminal, run the operator node, which acts as a service client.

```bash
ros2 run topic_service_action_rclcpp_example operator
```

### Running the Checker Node

Finally, run the checker node, which acts as an action client.

```bash
ros2 run topic_service_action_rclcpp_example checker
```

### Running the Launch File

You can run the argument and calculator nodes simultaneously using the launch file:

```bash
ros2 launch topic_service_action_rclcpp_example arithmetic.launch.py
```

## Conclusion

This tutorial covered the basic design and implementation of a ROS 2 package using topics, services, and actions. Future tutorials will provide more detailed explanations of the individual nodes and functionalities.

The source code used in this tutorial can be freely accessed in the [GitHub repository](https://github.com/robotpilot/ros2-seminar-examples).
