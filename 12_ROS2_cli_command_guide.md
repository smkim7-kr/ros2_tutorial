
# ROS 2 CLI Command Guide

## 1. Introduction to ROS 2 CLI

ROS 2 CLI (Command Line Interface) commands offer a way to interact with ROS 2 using the terminal. These commands allow you to perform various tasks, such as running nodes, managing packages, and interacting with topics, services, and actions. The structure of a typical ROS 2 CLI command is as follows:

```bash
ros2 [verb] [sub-verb] [options] [arguments]
```

In this guide, we'll cover some of the most commonly used ROS 2 CLI commands and their functionalities.

## 2. Common ROS 2 CLI Commands

### 2.1 `ros2 run`

The `ros2 run` command is used to run a specific node from a package.

```bash
$ ros2 run turtlesim turtlesim_node
```

This command runs the `turtlesim_node` from the `turtlesim` package.

### 2.2 `ros2 launch`

The `ros2 launch` command runs a launch file, which can start multiple nodes and set various configurations.

```bash
$ ros2 launch demo_nodes_cpp talker_listener.launch.py
```

This command runs the `talker_listener.launch.py` launch file from the `demo_nodes_cpp` package.

### 2.3 `ros2 pkg`

The `ros2 pkg` command is used to create new packages, list executables, and view package information.

```bash
$ ros2 pkg create my_first_ros_rclpy_pkg --build-type ament_python --dependencies rclpy std_msgs
```

This command creates a new package named `my_first_ros_rclpy_pkg` with dependencies on `rclpy` and `std_msgs`.

### 2.4 `ros2 node`

The `ros2 node` command provides information about running nodes.

```bash
$ ros2 node list
```

This command lists all currently running nodes.

### 2.5 `ros2 topic`

The `ros2 topic` command is used to interact with ROS topics, including publishing, subscribing, and getting information about topics.

```bash
$ ros2 topic echo /turtle1/cmd_vel
```

This command echoes messages being published to the `/turtle1/cmd_vel` topic.

### 2.6 `ros2 service`

The `ros2 service` command is used to interact with ROS services, allowing you to call services, list available services, and more.

```bash
$ ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 255, b: 255, width: 10}"
```

This command calls the `/turtle1/set_pen` service with specified parameters.

### 2.7 `ros2 action`

The `ros2 action` command is used to interact with ROS actions, which are long-running tasks that provide feedback and can be preempted.

```bash
$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.5708}"
```

This command sends a goal to the `/turtle1/rotate_absolute` action.

### 2.8 `ros2 interface`

The `ros2 interface` command provides information about the interfaces (messages, services, actions) used in ROS.

```bash
$ ros2 interface show geometry_msgs/msg/Twist
```

This command shows the structure of the `Twist` message from the `geometry_msgs` package.

### 2.9 `ros2 param`

The `ros2 param` command is used to interact with node parameters, including getting, setting, and listing parameters.

```bash
$ ros2 param get /turtlesim background_g
```

This command gets the value of the `background_g` parameter for the `turtlesim` node.

### 2.10 `ros2 bag`

The `ros2 bag` command is used for recording and playing back ROS messages.

```bash
$ ros2 bag record /turtle1/cmd_vel
```

This command starts recording messages being published to the `/turtle1/cmd_vel` topic.

## 3. Advanced ROS 2 CLI Usage

### 3.1 Using ROS Arguments with CLI

You can pass ROS-specific arguments when running commands, such as setting namespaces, node names, or remapping topics.

```bash
$ ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/tutorial -r __node:=my_turtle
```

This command runs the `turtlesim_node` with a custom namespace and node name.

### 3.2 Creating Custom ROS 2 CLI Commands

You can create custom ROS 2 CLI commands to extend the functionality of ROS 2. For example, the `ros2 env` command was created to manage environment variables.

#### Example Command: `ros2 env`

```bash
$ ros2 env list -a
```

This custom command lists all ROS-related environment variables.

## 4. Conclusion

The ROS 2 CLI provides powerful tools for interacting with and managing your ROS 2 environment. By becoming familiar with these commands, you can efficiently develop, debug, and manage your ROS 2 applications.
