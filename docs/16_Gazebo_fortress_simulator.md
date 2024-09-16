
# Simulating Robots with Gazebo and ROS2 Humble (Ubuntu 22.04)

## Overview
Simulation is a crucial part of robotics development, enabling developers to test algorithms without the risk, time, or expense of running them on real hardware. In ROS2 Humble on Ubuntu 22.04 (Jammy), you can use Gazebo Classic (Gazebo 11) or Gazebo Fortress (Ignition Gazebo) for robot simulations. This tutorial explains how to use both simulators with ROS2 Humble.

## Gazebo and Its Integration with ROS2 Humble
Gazebo is a powerful physics simulator where robots can be placed into simulated environments, allowing sensors and actuators to operate as they would in the real world. With Gazebo, simulated data can be published to ROS topics, and ROS nodes can control simulated actuators in real-time.

In ROS2 Humble, you have the option of using:
- **Gazebo Classic (Gazebo 11)** for existing workflows.
- **Gazebo Fortress** (also referred to as Ignition Gazebo), which is newer and provides more advanced features.

### Installing Gazebo for ROS2 Humble

#### Option 1: Install Gazebo Classic (Gazebo 11)
If you're migrating from ROS2 Foxy, you might want to continue using Gazebo Classic (Gazebo 11):
```bash
sudo apt install ros-humble-gazebo-ros-pkgs
```

#### Option 2: Install Gazebo Fortress (Ignition Gazebo)
For newer simulations, you can install **Gazebo Fortress**:
```bash
sudo apt install ros-humble-ros-gz
```

## Launching Gazebo in ROS2 Humble

### Using Gazebo Classic (Gazebo 11)
Once Gazebo Classic is installed, you can launch Gazebo using the same command as in ROS2 Foxy:
```bash
ros2 launch gazebo_ros gazebo.launch.py
```
You can start a predefined world by specifying the world file:
```bash
gazebo /usr/share/gazebo-11/worlds/seesaw.world
```

### Using Gazebo Fortress (Ignition Gazebo)
For launching **Gazebo Fortress**, use the new command:
```bash
ros2 launch ros_gz_sim gz_sim.launch.py
```
You can also specify a world file when launching:
```bash
ros2 launch ros_gz_sim gz_sim.launch.py world:=/path/to/world_file.sdf
```

## Gazebo Model Structure in ROS2 Humble
Gazebo uses **SDF (Simulation Description Format)**, which is slightly different from URDF. SDF describes not only robots but also the simulated world, including sensors, lights, and objects.

In ROS2, URDF models can be converted to SDF automatically for use in Gazebo. To spawn a robot in Gazebo from a URDF file, use the following command:
```bash
ros2 run gazebo_ros spawn_entity -entity my_robot -topic /robot_description
```
For Gazebo Fortress, the corresponding command is:
```bash
ros2 run ros_gz_sim spawn_entity -entity my_robot -topic /robot_description
```

## Gazebo Plugins for ROS2 Humble
Gazebo relies on various plugins to communicate with ROS. Some important plugins include:
- **Joint State Publisher**: Reads joint states from Gazebo and publishes them to the `/joint_states` topic.
- **Robot Control**: Allows ROS nodes to control the movement of robot joints in the simulation.
- **Sensor Plugins**: Simulate sensor data (e.g., cameras, LiDAR) and publish the data to ROS topics.

### Example of Gazebo Plugin (for Gazebo Classic and Fortress)
Here’s an example of a Gazebo plugin that simulates a camera sensor:

#### Gazebo Classic Plugin:
```xml
<gazebo>
  <sensor type="camera" name="my_camera">
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="gazebo_ros_camera" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <cameraName>my_camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
    </plugin>
  </sensor>
</gazebo>
```

#### Gazebo Fortress Plugin:
```xml
<gazebo>
  <sensor type="camera" name="my_camera">
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="ros_gz_camera" filename="libros_gz_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <cameraName>my_camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
    </plugin>
  </sensor>
</gazebo>
```

## Running Gazebo with ROS2 Humble
To run Gazebo and integrate it with ROS2, you typically use **launch files**. A launch file can initialize the Gazebo world, spawn a robot, and start all necessary plugins.

#### For Gazebo Classic:
```bash
ros2 launch gazebo_ros gazebo.launch.py
```

#### For Gazebo Fortress:
```bash
ros2 launch ros_gz_sim gz_sim.launch.py
```

Once running, you can control the robot in Gazebo via ROS topics, and visualize sensor data in RViz.

## Simulating Sensors in Gazebo for ROS2 Humble
Gazebo allows for the simulation of various sensors, including:
- **Cameras**: Simulate a camera's view and publish image data to ROS.
- **LiDAR**: Simulate laser scans and publish point cloud data.
- **Depth Cameras**: Provide depth images and point clouds.

### Example: Simulating a Depth Camera
#### Gazebo Classic Depth Camera:
```xml
<sensor type="depth" name="depth_camera">
  <plugin name="gazebo_ros_depth_camera" filename="libgazebo_ros_depth_camera.so"/>
</sensor>
```

#### Gazebo Fortress Depth Camera:
```xml
<sensor type="depth" name="depth_camera">
  <plugin name="ros_gz_depth_camera" filename="libros_gz_depth_camera.so"/>
</sensor>
```

This sensor will provide depth information as well as a point cloud, which can be visualized in RViz.

## Gazebo's Time Management in ROS2 Humble
When running simulations, it's important to understand that Gazebo controls time through the **/clock** topic. ROS nodes need to subscribe to this topic to synchronize with Gazebo’s time. To enable ROS nodes to use Gazebo’s simulated time, set the `use_sim_time` parameter in your ROS nodes:

```bash
ros2 param set /<node_name> use_sim_time true
```

You can include this parameter in your launch files to ensure all nodes synchronize with the simulated time.

## Conclusion
Gazebo is a versatile simulation tool that allows for realistic robot simulations with physics, sensors, and control. In ROS2 Humble, both Gazebo Classic and Gazebo Fortress offer extensive functionality. With plugins, URDF/SDF integration, and real-time sensor data publishing, Gazebo helps streamline the development process of robotic systems.

For further learning, check the [Articulated Robotics Gazebo Tutorial](https://articulatedrobotics.xyz/tutorials/ready-for-ros/gazebo/) or the official Gazebo documentation for ROS2 Humble.

