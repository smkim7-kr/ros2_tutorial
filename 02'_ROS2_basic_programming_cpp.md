
# ROS 2 Hello World: rclcpp Version

## 1. Introduction

When learning a programming language, the first thing we often encounter is the "Hello World" example, which outputs "Hello World" to the screen. In ROS, the "Hello World" example focuses more on message transmission. This tutorial demonstrates how to write and run a basic topic publisher and subscriber in ROS 2 using C++. The tutorial mirrors the `024 ROS Programming Basics (Python)` guide, allowing you to compare the differences and similarities between C++ and Python.

## 2. Creating a Package

Use the following command to create a ROS 2 package. The working directory should be the user's workspace directory as explained in the `020 ROS 2 File System` tutorial.

```bash
ros2 pkg create my_first_ros_rclcpp_pkg --build-type ament_cmake --dependencies rclcpp std_msgs
```

This command creates the package `my_first_ros_rclcpp_pkg` with dependencies on `rclcpp` and `std_msgs`. The directory structure of the package is as follows:

```
my_first_ros_rclcpp_pkg/
├── include/
│   └── my_first_ros_rclcpp_pkg/
├── src/
├── CMakeLists.txt
└── package.xml
```

## 3. Package Configuration

### 3.1 Package Configuration File (`package.xml`)

The `package.xml` file is configured based on the ROS 2 client libraries (RCL) used. For C++, set the `build_type` to `ament_cmake`.

```xml
<package format="3">
  <name>my_first_ros_rclcpp_pkg</name>
  <version>0.0.1</version>
  <description>ROS 2 rclcpp basic package</description>
  <maintainer email="example@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 3.2 Build Configuration File (`CMakeLists.txt`)

The `CMakeLists.txt` file specifies dependencies and the build/installation setup.

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_first_ros_rclcpp_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(helloworld_publisher src/helloworld_publisher.cpp)
ament_target_dependencies(helloworld_publisher rclcpp std_msgs)

add_executable(helloworld_subscriber src/helloworld_subscriber.cpp)
ament_target_dependencies(helloworld_subscriber rclcpp std_msgs)

install(TARGETS
  helloworld_publisher
  helloworld_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

## 4. Writing the Publisher Node

Create the `helloworld_publisher.cpp` file in the `src` directory:

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class HelloworldPublisher : public rclcpp::Node
{
public:
  HelloworldPublisher()
  : Node("helloworld_publisher"), count_(0)
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    helloworld_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "helloworld", qos_profile);
    timer_ = this->create_wall_timer(
      1s, std::bind(&HelloworldPublisher::publish_helloworld_msg, this));
  }

private:
  void publish_helloworld_msg()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello World: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Published message: '%s'", msg.data.c_str());
    helloworld_publisher_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr helloworld_publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HelloworldPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

## 5. Writing the Subscriber Node

Create the `helloworld_subscriber.cpp` file in the `src` directory:

```cpp
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class HelloworldSubscriber : public rclcpp::Node
{
public:
  HelloworldSubscriber()
  : Node("Helloworld_subscriber")
  {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    helloworld_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "helloworld",
      qos_profile,
      std::bind(&HelloworldSubscriber::subscribe_topic_message, this, _1));
  }

private:
  void subscribe_topic_message(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr helloworld_subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HelloworldSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

## 6. Building the Package

Build the package using `colcon`:

```bash
cd ~/robot_ws
colcon build --symlink-install --packages-select my_first_ros_rclcpp_pkg
. ~/robot_ws/install/local_setup.bash
```

## 7. Running the Nodes

Run the publisher and subscriber nodes:

```bash
ros2 run my_first_ros_rclcpp_pkg helloworld_publisher
ros2 run my_first_ros_rclcpp_pkg helloworld_subscriber
```

## 8. Conclusion

This tutorial demonstrated how to create, build, and run a basic ROS 2 package with a topic publisher and subscriber using C++. By comparing this with the Python example, you can understand the differences and similarities between the two languages in the context of ROS 2.
