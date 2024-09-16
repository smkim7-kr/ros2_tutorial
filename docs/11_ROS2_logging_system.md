
# ROS 2 Logging System

## 1. Introduction to Logging

When learning a new programming language, it's common to print "Hello World!" to the terminal as a first step. In ROS 2, logging is an essential tool used during development for debugging and monitoring. The logging system in ROS 2 is simple, powerful, and provides several features:

- **Simple Interface**: Easy to use without much setup.
- **No Initialization Needed**: Ready to use out of the box.
- **Multiple Log Levels**: DEBUG, INFO, WARN, ERROR, FATAL.
- **Advanced Filtering**: Supports conditional logging, one-time logging, throttled logging, etc.
- **Performance**: Designed to have minimal impact on runtime performance.
- **Thread-Safe**: Ensures safe logging in multi-threaded environments.
- **Rich Output Information**: Includes file names, line numbers, node names, and namespaces.
- **Launch File Integration**: Log levels can be configured via launch files.
- **Runtime Log Level Change**: Log levels can be changed during runtime.

## 2. Configuring Logs

### 2.1 Log Directory
Logs are stored in the `~/.ros/log` directory by default. In ROS 2 Foxy, this path is fixed, but from Galactic onwards, the log directory can be changed.

### 2.2 Log Levels

ROS 2 supports five log levels:

1. **DEBUG**: Detailed information for diagnosing issues.
2. **INFO**: General information about the node’s operation.
3. **WARN**: Indicates potential problems or important events.
4. **ERROR**: Indicates a definite problem in the node.
5. **FATAL**: Indicates a critical issue causing the node to stop.

Log levels can be set programmatically in the code, externally via services, or through the command line.

**Programmatically**: Use macros in RCLCPP or methods in RCLPY to set the log level within the code.

```cpp
// C++ example
RCLCPP_INFO(this->get_logger(), "Published message: '%s'", msg.data.c_str());
```

```python
# Python example
self.get_logger().info('Published message: {0}'.format(msg.data))
```

**Externally**: ROS 1 allowed changing log levels via `rqt_logger_level`. In ROS 2, similar functionality is planned but not yet fully implemented.

**Command Line**: Set the log level when running a node:

```bash
$ ros2 run logging_demo logging_demo_main --ros-args --log-level debug
```

### 2.3 Console Output Formatting

The format of log messages in the console can be customized using environment variables. For example:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})"
```

### 2.4 Console Output Colorizing

Logs are colorized based on severity. For example, INFO logs are white, WARN logs are yellow, and ERROR logs are red. This can be controlled with the environment variable:

```bash
export RCUTILS_COLORIZED_OUTPUT=1
```

### 2.5 Stream for Console Output

From Foxy onwards, all logs are output to `stderr` by default, but this can be changed to `stdout` if needed:

```bash
export RCUTILS_LOGGING_USE_STDOUT=1
```

### 2.6 Line Buffered Console Output

By default, INFO and DEBUG logs are not line-buffered. To enable line buffering:

```bash
export RCUTILS_LOGGING_BUFFERED_STREAM=1
```

## 3. Example Code

ROS 2 includes example logging code in the `logging_demo` package. Below is an overview of how logging is implemented in both C++ (RCLCPP) and Python (RCLPY).

### 3.1 RCLCPP Example

The C++ example demonstrates using various log levels and features like one-time logging and conditional logging.

```cpp
RCLCPP_INFO_ONCE(get_logger(), "This logs only once");
RCLCPP_DEBUG_FUNCTION(get_logger(), &debug_function_to_evaluate_, "Debug message");
```

### 3.2 RCLPY Example

The Python example mirrors the C++ functionality, but with Python's logging tools. It lacks some advanced features present in RCLCPP but provides a similar structure.

```python
self.get_logger().log('Timer callback called', LoggingSeverity.INFO, once=True)
```

## 4. Conclusion

Logging in ROS 2 is a fundamental tool for debugging and monitoring node behavior. By understanding and utilizing the various logging levels and configurations, developers can create more reliable and maintainable systems.
