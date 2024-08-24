
# ROS Package Design

## 1. ROS Package Design
The difference between robot programs integrated with ROS and typical robot programs is the need to design the process into nodes, with communication between these nodes in mind. This tutorial introduces an example of ROS 2 programming using topics, services, and actions (in Python). The design integrates these components to work together rather than separately within a single package.

As shown in Figure 1, the design involves four nodes, each utilizing a topic, service, and action. Each node may include one or more topic publishers, subscribers, service servers, clients, action servers, and clients. The central node plays a critical role in coordinating the other nodes, and additional components like parameters, arguments, and launch files are incorporated for future use.

![Package Design Overview](#) 

The package is named `topic_service_action_rclpy_example`, and each node, topic, service, and action has a unique name, as depicted in Figure 2.

### Process Overview:
1. **Argument Node**: Publishes the current time and variables a and b under the topic `arithmetic_argument`.
2. **Calculator Node**: Subscribes to the topic and retrieves the time and variables a and b.
3. **Operator Node**: Sends an operator (+, -, *, /) as a service request to the calculator node under the service `arithmetic_operator`.
4. **Calculator Node**: Performs the calculation using the operator received from the operator node and sends the result back as a service response.
5. **Checker Node**: Sends a goal sum as an action goal, after which the calculator node sums the results and provides feedback with each calculation formula. When the goal sum is exceeded, it sends the final result.

![Detailed Package Design](#)

## 2. Node Implementation
The `topic_service_action_rclpy_example` package consists of four nodes: argument, operator, calculator, and checker. The source code for these nodes is available in the [GitHub repository](#). Detailed explanations of each node's implementation will be provided in subsequent tutorials.

## 3. Package Configuration File (package.xml)
The `package.xml` file for the `topic_service_action_rclpy_example` package is shown below:

```xml
<package format="3">
  <name>topic_service_action_rclpy_example</name>
  <version>0.2.0</version>
  <description>ROS 2 rclpy example package for the topic, service, action</description>
  <maintainer email="passionvirus@gmail.com">Pyo</maintainer>
  <license>Apache License 2.0</license>
  <author email="passionvirus@gmail.com">Pyo</author>
  <author email="routiful@gmail.com">Darby Lim</author>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>msg_srv_action_interface_example</depend>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

The key dependency is on the `msg_srv_action_interface_example` package, which was covered in a previous tutorial on topic, service, and action interfaces.

## 4. Python Package Setup File (setup.py)
The `setup.py` file for the `topic_service_action_rclpy_example` package is shown below:

```python
#!/usr/bin/env python3

import glob
import os
from setuptools import find_packages, setup

package_name = 'topic_service_action_rclpy_example'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        (share_dir + '/param', glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Pyo, Darby Lim',
    author_email='passionvirus@gmail.com, routiful@gmail.com',
    maintainer='Pyo',
    maintainer_email='passionvirus@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS 2 rclpy example package for the topic, service, action',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'argument = topic_service_action_rclpy_example.arithmetic.argument:main',
            'operator = topic_service_action_rclpy_example.arithmetic.operator:main',
            'calculator = topic_service_action_rclpy_example.calculator.main:main',
            'checker = topic_service_action_rclpy_example.checker.main:main',
        ],
    },
)
```

### Key Points:

#### 4.1 data_files
The `data_files` section lists files used by the package, ensuring they are included when the package is installed. This includes files like `package.xml`, launch files, and parameter files.

#### 4.2 entry_points
The `entry_points` section defines console scripts for the nodes, enabling their execution using commands like `ros2 run`.

## 5. Source Code Download and Build
To download and build the source code:

```bash
$ cd ~/robot_ws/src
$ git clone https://github.com/robotpilot/ros2-seminar-examples.git
$ cd ~/robot_ws && colcon build --symlink-install
```

Or use the predefined aliases:

```bash
$ cw
$ cbp topic_service_action_rclpy_example
```

After a successful build, the necessary files will be located in the `~/robot_ws/install/topic_service_action_rclpy_example` directory.

## 6. Execution

### 6.1 Run the Calculator Node
First, run the calculator node, which acts as a subscriber, service server, and action server. It will enter a standby mode, waiting for further actions.

```bash
$ ros2 run topic_service_action_rclpy_example calculator
```

### 6.2 Run the Argument Node
Next, run the argument node, which publishes arguments a and b. The calculator node will display the received data.

```bash
$ ros2 run topic_service_action_rclpy_example argument
```

### 6.3 Run the Operator Node
The operator node sends a randomly chosen operator to the calculator node and displays the calculated result.

```bash
$ ros2 run topic_service_action_rclpy_example operator
```

### 6.4 Run the Checker Node
The checker node sends a goal sum as an action goal. The calculator node then sums the results and sends feedback and the final result when the goal is reached.

```bash
$ ros2 run topic_service_action_rclpy_example checker
```

You can modify the goal sum by passing the `-g` argument:

```bash
$ ros2 run topic_service_action_rclpy_example checker -g 100
```

### 6.5 Run with a Launch File
To run the argument and calculator nodes together, use the launch file:

```bash
$ ros2 launch topic_service_action_rclpy_example arithmetic.launch.py
```

## 7. Conclusion
This guide covered the design, setup, build, and execution of a ROS 2 package integrating topics, services, and actions. The next tutorials will delve into the code behind topics, services, actions, parameters, and launch files. The source code is available in the [GitHub repository](#), and additional examples can be found in the official ROS demos.
