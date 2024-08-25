
# ROS 2 Command-Line Arguments Programming

---

#### 1. Command-Line Arguments

Command-line arguments are parameters added to a program's execution command to influence its behavior at runtime. In ROS 2, these arguments are processed within the `main` function of a node.

For example, in a previous lecture (`028 ROS 2 Package Design (Python)`), the following command was used to set the `GOAL_TOTAL_SUM` to 100. The `ros2 run` command initiates the `checker` node from the `topic_service_action_rclpy_example` package. The additional argument `-g 100` is a command-line argument that modifies the node's behavior during execution.

```bash
$ ros2 run topic_service_action_rclpy_example checker -g 100
```

- **Parameter**: Refers to variables defined within a function.
- **Argument**: Refers to the values provided during function calls.

#### 2. Handling Command-Line Arguments in ROS 2

In ROS 2, command-line arguments are processed similarly to how they are handled in C++ and Python programs. In C++, arguments are passed as `argc` and `argv` to the `main` function and then forwarded to the `rclcpp::init` function.

```cpp
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  // ...
}
```

In Python, arguments can be ignored by setting `args=None` and passing it directly to `rclpy.init`. If arguments need to be used, they are stored in `argv` after removing the first element (which is the program's name).

```python
def main(args=None):
    rclpy.init(args=args)
    # ...
```

If you want to use command-line arguments, you need to parse them using Python's `argparse` module.

```python
def main(argv=sys.argv[1:]):
    # Argument parsing code
    rclpy.init(args=argv)
    # ...
```

#### 3. Parsing Command-Line Arguments

The parsing of command-line arguments is implemented in the `Checker` node's `main` function. Below is the full code example.

```python
import argparse
import sys
import rclpy
from topic_service_action_rclpy_example.checker.checker import Checker

def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-g',
        '--goal_total_sum',
        type=int,
        default=50,
        help='Target goal value of total sum')
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    args = parser.parse_args()

    rclpy.init(args=args.argv)
    try:
        checker = Checker()
        checker.send_goal_total_sum(args.goal_total_sum)
        try:
            rclpy.spin(checker)
        except KeyboardInterrupt:
            checker.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            checker.arithmetic_action_client.destroy()
            checker.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

##### Steps to Parse Arguments:

1. **Create the parser**: Using `argparse.ArgumentParser`, an argument parser object is created.
2. **Add arguments**: Use the `add_argument()` method to specify the arguments to be used, their types, default values, and descriptions.
3. **Parse arguments**: Use `parse_args()` to parse the command-line arguments.
4. **Use arguments**: Access the parsed arguments using `args.<argument_name>`.

##### Example Breakdown:

1. **Create the parser**:
   ```python
   parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
   ```

2. **Add arguments**:
   ```python
   parser.add_argument(
       '-g',
       '--goal_total_sum',
       type=int,
       default=50,
       help='Target goal value of total sum')
   ```

3. **Parse arguments**:
   ```python
   args = parser.parse_args()
   ```

4. **Use arguments**:
   ```python
   checker.send_goal_total_sum(args.goal_total_sum)
   ```

#### 4. Conclusion

This tutorial covered the basics of command-line argument programming in ROS 2. The code used in this tutorial is available in the following GitHub repository. For further details on the functions used, refer to the RCLPY API documentation.

- **GitHub Repository**: [topic_service_action_rclpy_example](https://github.com/robotpilot/ros2-seminar-examples)

---
