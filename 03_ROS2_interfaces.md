# Creating a New ROS 2 Interface

## 1. Writing a New ROS 2 Interface

In ROS 2 programming, communication between nodes typically uses existing interfaces like `std_msgs` for basic data types (integer, floating point, boolean), `geometry_msgs/Twist` for velocities, or `sensor_msgs/LaserScan` for laser scan data. However, these interfaces may not cover all user needs, especially for service and action interfaces, which often require custom definitions.

This guide will demonstrate how to create new topic, service, and action interfaces in ROS 2 for basic programming examples.

### Consideration:

While interfaces can be included in the same package as the nodes that use them, it's generally better to create a separate package dedicated solely to interfaces. This simplifies dependency management. For example, if package A uses interface a, and packages B and C also need to use a, then B and C must depend on package A. Creating a separate interface package avoids this issue.

In this guide, we'll create a package named `msg_srv_action_interface_example` containing the following:

- `ArithmeticArgument.msg`
- `ArithmeticOperator.srv`
- `ArithmeticChecker.action`

## 2. Creating the Interface Package

Navigate to the workspace's `src` folder and create the package using the `ros2 pkg create` command. Then create folders for the interfaces.

```bash
$ cd ~/robot_ws/src
$ ros2 pkg create --build-type ament_cmake msg_srv_action_interface_example
$ cd msg_srv_action_interface_example
$ mkdir msg srv action
```

Next, create the following interface files in the corresponding folders:

- `ArithmeticArgument.msg` in `msg/`
- `ArithmeticOperator.srv` in `srv/`
- `ArithmeticChecker.action` in `action/`

### 2.1 ArithmeticArgument.msg

Location: `msg_srv_action_interface_example/msg`

```plaintext
# Messages
builtin_interfaces/Time stamp
float32 argument_a
float32 argument_b
```

### 2.2 ArithmeticOperator.srv

Location: `msg_srv_action_interface_example/srv`

```plaintext
# Constants
int8 PLUS = 1
int8 MINUS = 2
int8 MULTIPLY = 3
int8 DIVISION = 4

# Request
int8 arithmetic_operator
---
# Response
float32 arithmetic_result
```

### 2.3 ArithmeticChecker.action

Location: `msg_srv_action_interface_example/action`

```plaintext
# Goal
float32 goal_sum
---
# Result
string[] all_formula
float32 total_sum
---
# Feedback
string[] formula
```

### 2.4 Comparison of msg, srv, and action Interfaces

| Interface Type | File Extension | Data                     | Example Fields             |
| -------------- | -------------- | ------------------------ | -------------------------- |
| msg            | *.msg          | Topic data               | `float32 argument_a`       |
| srv            | *.srv          | Service request/response | `int8 arithmetic_operator` |
| action         | *.action       | Goal, result, feedback   | `float32 goal_sum`         |

## 3. Package Configuration File (package.xml)

Create the `package.xml` file for the `msg_srv_action_interface_example` package. Adjust the contents according to your specific needs.

```xml
<package format="3">
  <name>msg_srv_action_interface_example</name>
  <version>0.2.0</version>
  <description>
    ROS 2 example for message, service and action interface
  </description>
  <maintainer email="passionvirus@gmail.com">Pyo</maintainer>
  <license>Apache 2.0</license>
  <author email="passionvirus@gmail.com">Pyo</author>
  <author email="routiful@gmail.com">Darby Lim</author>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>builtin_interfaces</exec_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Key Points:

- `rosidl_default_generators` is used during the build process to generate Interface Definition Language (IDL) files.
- The package also depends on `builtin_interfaces` and `rosidl_default_runtime`.

## 4. Build Configuration File (CMakeLists.txt)

Create the `CMakeLists.txt` file for building the `msg_srv_action_interface_example` package.

```cmake
cmake_minimum_required(VERSION 3.5)
project(msg_srv_action_interface_example)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/ArithmeticArgument.msg"
)

set(srv_files
  "srv/ArithmeticOperator.srv"
)

set(action_files
  "action/ArithmeticChecker.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

### Key Points:

- The `set` commands list the interface files.
- `rosidl_generate_interfaces` processes these files and generates the necessary code.

## 5. Building the Package

To build the package, navigate to the workspace and use the following command:

```bash
$ cw
$ cbp msg_srv_action_interface_example
```

After a successful build, the generated interface files (e.g., *.h, *.hpp, *.py, *.idl) will be located in the `~/robot_ws/install/msg_srv_action_interface_example` directory.

## 6. Conclusion

Creating an interface package in ROS 2 is straightforward. You just need to write the interface files, place them in the appropriate directories, and configure the `package.xml` and `CMakeLists.txt` files. In a future guide, we'll explore how to use these interfaces in a ROS 2 program. The source code for this exercise is available on the accompanying [GitHub repository](#).