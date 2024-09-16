
# ROS 2 Launch System

In ROS 2, the `ros2 run` command is used to execute a single node, which works fine for running individual nodes. However, in many cases, it's more common to run multiple nodes simultaneously to allow them to communicate with each other. This often involves not just custom-developed nodes but also using nodes from publicly available packages, along with various options.

To address this need, ROS 2 provides a concept called `launch`, which allows the execution of one or more predefined nodes. Additionally, when launching nodes, you can use options such as changing package parameters, renaming nodes, setting node namespaces, and modifying environment variables. In ROS 1, this was handled by `roslaunch`, which used XML-based `*.launch` files to set up node execution. In ROS 2, Python-based launch files have been introduced to offer more flexibility and functionality, although XML and YAML formats are also supported.

*Note: While ROS 2 supports XML and YAML launch files, this guide focuses on the Python format, which is more versatile.*

## Creating a Launch File

Let's create a new launch file in the `topic_service_action_rclpy_example` package. This launch file will primarily start the `argument` and `calculator` nodes and specify the parameter file these nodes will use.

First, ensure that your package contains a `launch` directory, where you can place the `*.launch.py` file. For this example, we'll create a file named `arithmetic.launch.py` in the following location:

```plaintext
topic_service_action_rclpy_example/launch/arithmetic.launch.py
```

Here’s the complete source code for `arithmetic.launch.py`:

```python
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('topic_service_action_rclpy_example'),
            'param',
            'arithmetic_config.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path of parameter file'),

        Node(
            package='topic_service_action_rclpy_example',
            executable='argument',
            name='argument',
            parameters=[param_dir],
            output='screen'),

        Node(
            package='topic_service_action_rclpy_example',
            executable='calculator',
            name='calculator',
            parameters=[param_dir],
            output='screen'),
    ])
```

Let's break down the key elements:

1. **Launch Configuration:**
   - The `LaunchConfiguration` class is used to set up configurations for the launch. Here, we define `param_dir`, which points to the `arithmetic_config.yaml` file in the `param` folder of the `topic_service_action_rclpy_example` package.

2. **Launch Description:**
   - The `LaunchDescription` class is used to return a list of actions, such as declaring launch arguments and starting nodes.
   - `DeclareLaunchArgument`: This declares `param_dir` as a launch argument.
   - `Node`: This class is used to define the nodes to be executed. Here, it specifies the package, executable, node name, parameters, and output configuration.

When you run this launch file, it will start the `argument` and `calculator` nodes from the `topic_service_action_rclpy_example` package.

## Additional Launch Features

1. **Remappings:**
   - You can remap topics, services, and actions using the `remappings` feature. For example, you can remap the `/arithmetic_argument` topic to `/argument` as shown below:

    ```python
    Node(
        package='topic_service_action_rclpy_example',
        executable='argument',
        name='argument',
        remappings=[
            ('/arithmetic_argument', '/argument'),
        ]
    )
    ```

2. **Namespace:**
   - The `namespace` feature allows you to group nodes, topics, services, and other resources under a common namespace, which is useful when dealing with multiple robots or complex systems.

    ```python
    ros_namespace = LaunchConfiguration('ros_namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_namespace',
            default_value=os.environ['ROS_NAMESPACE'],
            description='Namespace for the robot'),

        Node(
            package='topic_service_action_rclpy_example',
            namespace=ros_namespace,
            executable='argument',
            name='argument',
            output='screen'),
    ])
    ```

3. **Including Other Launch Files:**
   - You can modularize your launch files by including other launch files. This is particularly useful when working with multiple packages or complex systems.

    ```python
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import ThisLaunchFileDir

    def generate_launch_description():
        return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [ThisLaunchFileDir(), '/other_launch_file.launch.py']),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [get_package_share_directory('another_package'), '/launch/another_launch_file.launch.py']),
            ),
        ])
    ```

## Building the Package

To use the launch files within the ROS 2 ecosystem, they need to be installed in the appropriate location, which requires building the package.

1. **RCLCPP Packages:**
   - For C++ packages, add the following to the `CMakeLists.txt` file:

    ```cmake
    install(DIRECTORY
      launch
      DESTINATION share/${PROJECT_NAME}/
    )
    ```

2. **RCLPY Packages:**
   - For Python packages, add the following to the `setup.py` file:

    ```python
    setup(
        name=package_name,
        version='0.1.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            (share_dir, ['package.xml']),
            (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
            (share_dir + '/param', glob.glob(os.path.join('param', '*.yaml'))),
        ],
    ```

3. **Building:**
   - Use the following command to build the package:

    ```bash
    $ cw
    $ cbp topic_service_action_rclpy_example
    ```

## Running the Launch File

To run the launch file, use the `ros2 launch` command:

```bash
ros2 launch topic_service_action_rclpy_example arithmetic.launch.py
```

## Conclusion

This guide provided an overview of using launch files in ROS 2. The original code used in these examples can be found in the [ROS 2 Seminar Examples GitHub repository](https://github.com/robotpilot/ros2-seminar-examples). For more information on the functions used, refer to the RCLPY API documentation.
