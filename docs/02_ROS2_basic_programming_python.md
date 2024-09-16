# ROS Hello World: rclpy Version

## 1. ROS Hello World, rclpy Version

When learning a programming language, the first example is often a "Hello World" program, which simply prints "Hello World" to the screen. In ROS, the Hello World example is slightly different, focusing on message transmission rather than just output. This guide walks through writing and running the simplest structure of a topic publisher and subscriber in ROS 2 using Python.

## 2. Creating a Package

To create a ROS 2 package, use the `ros2 pkg create` command followed by options. The working directory should be the user’s workspace as described in the ROS 2 file system structure guide.

```bash
$ cd ~/robot_ws/src/
$ ros2 pkg create my_first_ros_rclpy_pkg --build-type ament_python --dependencies rclpy std_msgs
```

The dependencies `rclpy` and `std_msgs` indicate that the package will use the standard ROS message package and the Python client library for ROS. The package can be created with these dependencies or added later in the `package.xml` file. Once the package is created, it will include a basic directory structure and files like `package.xml`, `setup.py`, etc.

## 3. Configuring the Package

### 3.1 Package Configuration File (package.xml)

The `package.xml` file specifies the package’s dependencies and configuration. For Python, the `build_type` is set to `ament_python`.

```xml
<package format="3">
  <name>my_first_ros_rclpy_pkg</name>
  <version>0.0.2</version>
  <description>ROS 2 rclpy basic package for the ROS 2 seminar</description>
  <maintainer email="pyo@robotis.com">Pyo</maintainer>
  <license>Apache License 2.0</license>
  <author email="mikael@osrfoundation.org">Mikael Arguedas</author>
  <author email="pyo@robotis.com">Pyo</author>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 3.2 Python Package Setup File (setup.py)

The `setup.py` file is crucial for defining how the package is built and installed. The `entry_points` section defines console scripts for the publisher and subscriber.

```python
from setuptools import find_packages
from setuptools import setup

package_name = 'my_first_ros_rclpy_pkg'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mikael Arguedas, Pyo',
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    entry_points={
        'console_scripts': [
            'helloworld_publisher = my_first_ros_rclpy_pkg.helloworld_publisher:main',
            'helloworld_subscriber = my_first_ros_rclpy_pkg.helloworld_subscriber:main',
        ],
    },
)
```

### 3.3 Python Package Configuration File (setup.cfg)

This file configures where the scripts will be installed, setting the script directory.

```ini
[develop]
script-dir=$base/lib/my_first_ros_rclpy_pkg
[install]
install-scripts=$base/lib/my_first_ros_rclpy_pkg
```

## 4. Writing the Publisher Node

The publisher node is written in Python and saved as `helloworld_publisher.py` in the package directory. It sets up a simple publisher that sends "Hello World" messages at regular intervals.

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class HelloworldPublisher(Node):

    def __init__(self):
        super().__init__('helloworld_publisher')
        qos_profile = QoSProfile(depth=10)
        self.helloworld_publisher = self.create_publisher(String, 'helloworld', qos_profile)
        self.timer = self.create_timer(1, self.publish_helloworld_msg)
        self.count = 0

    def publish_helloworld_msg(self):
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.helloworld_publisher.publish(msg)
        self.get_logger().info(f'Published message: {msg.data}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = HelloworldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. Writing the Subscriber Node

Similarly, the subscriber node is written in Python and saved as `helloworld_subscriber.py`. It subscribes to the "Hello World" messages and logs them.

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

class HelloworldSubscriber(Node):

    def __init__(self):
        super().__init__('Helloworld_subscriber')
        qos_profile = QoSProfile(depth=10)
        self.helloworld_subscriber = self.create_subscription(
            String,
            'helloworld',
            self.subscribe_topic_message,
            qos_profile)

    def subscribe_topic_message(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = HelloworldSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6. Building the Package

To build the package, use the `colcon` build tool. The `--symlink-install` option is often used to avoid copying files unnecessarily.

```bash
$ cd ~/robot_ws
$ colcon build --symlink-install --packages-select my_first_ros_rclpy_pkg
```

After the first build, source the setup file to configure the environment for running the nodes.

```bash
. ~/robot_ws/install/local_setup.bash
```

## 7. Running the Nodes

Run the publisher and subscriber nodes using the `ros2 run` command.

```bash
$ ros2 run my_first_ros_rclpy_pkg helloworld_subscriber
$ ros2 run my_first_ros_rclpy_pkg helloworld_publisher
```

## 8. Conclusion

This guide provided a basic introduction to creating, configuring, and running a simple ROS 2 package with a publisher and subscriber using Python. The next step is to create a similar example using C++ with rclcpp.