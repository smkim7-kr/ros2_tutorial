
# ROS 2 Parameter Programming

## 1. Parameters in ROS 2

Parameters in ROS 2 allow nodes to set, get, and change internal configurations dynamically. Unlike services that handle requests and responses, parameters provide a way to modify node behavior without modifying code, typically used for configurations like QoS settings or other node-specific settings.

## 2. Parameter Setup in Nodes

Let's explore how parameters are set up and used in the `argument` node. Here are three main functions used to handle parameters:

1. **`declare_parameter`**: This function is used to declare parameters with a name and a default value.
2. **`get_parameter`**: This function retrieves the value of a parameter.
3. **`add_on_set_parameters_callback`**: This function sets up a callback for when parameters are changed.

Example from the `Argument` class:

```python
class Argument(Node):
    def __init__(self):
        super().__init__('argument')
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value
        self.declare_parameter('min_random_num', 0)
        self.min_random_num = self.get_parameter('min_random_num').value
        self.declare_parameter('max_random_num', 9)
        self.max_random_num = self.get_parameter('max_random_num').value
        self.add_on_set_parameters_callback(self.update_parameter)

    def update_parameter(self, params):
        for param in params:
            if param.name == 'min_random_num' and param.type_ == Parameter.Type.INTEGER:
                self.min_random_num = param.value
            elif param.name == 'max_random_num' and param.type_ == Parameter.Type.INTEGER:
                self.max_random_num = param.value
        return SetParametersResult(successful=True)
```

### Explanation:

- `declare_parameter`: Declares parameters like `qos_depth`, `min_random_num`, and `max_random_num` with default values.
- `get_parameter`: Retrieves the declared parameters' values, useful when setting initial configurations or fetching values from a parameter file.
- `add_on_set_parameters_callback`: Sets up a callback function `update_parameter`, which is triggered when parameters are changed.

In the `publish_random_arithmetic_arguments` method, parameters are used to define the range for the random number generator:

```python
def publish_random_arithmetic_arguments(self):
    msg = ArithmeticArgument()
    msg.stamp = self.get_clock().now().to_msg()
    msg.argument_a = float(random.randint(self.min_random_num, self.max_random_num))
    msg.argument_b = float(random.randint(self.min_random_num, self.max_random_num))
    self.arithmetic_argument_publisher.publish(msg)
    self.get_logger().info('Published argument a: {0}'.format(msg.argument_a))
    self.get_logger().info('Published argument b: {0}'.format(msg.argument_b))
```

## 3. Using Parameters with CLI

Once parameters are set up, you can use the ROS 2 CLI to interact with them. Here's how you can list, get, and set parameters for a running node:

1. **List parameters**: Use `ros2 param list` to see all parameters for a node.
    ```bash
    $ ros2 param list /argument
    ```

2. **Get parameter values**: Use `ros2 param get` to retrieve the value of a specific parameter.
    ```bash
    $ ros2 param get /argument max_random_num
    ```

3. **Set parameter values**: Use `ros2 param set` to change the value of a parameter.
    ```bash
    $ ros2 param set /argument max_random_num 100
    ```

After setting the `max_random_num` parameter to 100, you should see the node publishing values within this new range.

## 4. Using Parameters Programmatically

Parameters can also be modified programmatically from another node using the `SetParameters` service, as shown below:

```python
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters

self.random_num_parameter_client = self.create_client(
    SetParameters,
    'argument/set_parameters')

def set_max_random_num_parameter(self, max_value):
    request = SetParameters.Request()
    parameter = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=max_value)
    request.parameters = [Parameter(name='max_random_num', value=parameter)]
    service_client = self.random_num_parameter_client
    return self.call_service(service_client, request, 'max_random_num parameter')
```

### Explanation:

- A service client is created to connect to the `set_parameters` service of the `argument` node.
- The `set_max_random_num_parameter` method constructs a parameter change request and sends it to the service.

## 5. Setting Default Parameters

For large projects, managing many parameters can be cumbersome. Using a YAML file to predefine parameters is more efficient:

Example `arithmetic_config.yaml`:

```yaml
/**:
  ros__parameters:
    qos_depth: 30
    min_random_num: 0
    max_random_num: 9
```

This file is referenced in a launch file, which loads these parameters when the node starts:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('topic_service_action_rclpy_example'),
            'param',
            'arithmetic_config.yaml'))

    return LaunchDescription([
        Node(
            package='topic_service_action_rclpy_example',
            executable='argument',
            name='argument',
            parameters=[param_dir],
            output='screen'),
    ])
```

## 6. Conclusion

This section covered how to use parameters in ROS 2 for dynamic node configuration. The next tutorial will explore launch files and other advanced topics. For further information, refer to the original code examples and ROS 2 documentation.

[ROS 2 Seminar Examples GitHub Repository](https://github.com/robotpilot/ros2-seminar-examples)
