
# Driving Your Virtual Robot in Gazebo with ROS2

## Overview
In this tutorial, we continue developing the concept design of our differential drive robot by simulating it in **Gazebo**. By the end of this guide, you will have a virtual version of your robot that you can drive around in a simulated environment.

This tutorial builds upon our previous URDF file. If you haven't seen that one yet, make sure to check it out first to understand how we created the basic structure of the robot.

## Prerequisites
Before starting, ensure that you have:
1. Installed **ROS2**.
2. Installed **Gazebo** and integrated it with ROS2:
   ```bash
   sudo apt install ros-humble-gazebo-ros-pkgs
   ```

Make sure you have also completed the following setup:
- Created the **URDF** file for your robot.
- Installed and set up ROS2 workspaces with `colcon` and sourced them.

## Steps to Simulate the Robot in Gazebo

### 1. Running Robot State Publisher in Simulation Mode
We will begin by running the **Robot State Publisher**, just like before, but now in **simulation mode** with Gazebo. Use the following command:
```bash
ros2 launch <your_package> rsp.launch.py use_sim_time:=true
```
Here, the `use_sim_time:=true` ensures that the node synchronizes with Gazebo’s internal clock.

### 2. Launching Gazebo with ROS Compatibility
Now, launch **Gazebo** using the provided launch file in the `gazebo_ros` package:
```bash
ros2 launch gazebo_ros gazebo.launch.py
```
This will start Gazebo with ROS integration, allowing your simulated robot to communicate with ROS topics.

### 3. Spawning the Robot in Gazebo
Finally, use the `spawn_entity.py` script to spawn the robot into the Gazebo environment. This script is included in the `gazebo_ros` package:
```bash
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity bot_name
```
This command reads the URDF published by **Robot State Publisher** on the `/robot_description` topic and spawns the robot in the Gazebo environment.

## Creating a Launch File for Simplicity
Instead of running these commands individually, it is better to wrap them in a single launch file. This simplifies relaunching the simulation whenever you make changes.

Here’s a sample launch file, `sim.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-topic', '/robot_description', '-entity', 'bot_name'],
        ),
        Node(
            package='<your_package_name>', executable='rsp.launch.py',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='gazebo_ros', executable='gazebo.launch.py',
        ),
    ])
```
After saving this file, run it using the following command:
```bash
ros2 launch <your_package> sim.launch.py
```

## Gazebo Tags for Simulation Enhancements

### 1. Adding Colors to the Robot in Gazebo
Gazebo requires **special tags** to define materials and colors. For each link in your URDF, add a **gazebo tag** to specify the color:
```xml
<gazebo reference="chassis">
  <material>Gazebo/White</material>
</gazebo>
```
Repeat this for each link, using the correct color names. Example colors include `Gazebo/White`, `Gazebo/Blue`, and `Gazebo/Black`.

### 2. Controlling the Robot Using ROS2 Commands
We can now control the robot using a **differential drive** controller. For simplicity, we will use **Gazebo’s built-in plugin** to control the robot before transitioning to the full **ROS2 Control** setup in future tutorials.

Create a new file called `gazebo_control.xacro` and include the following plugin:
```xml
<gazebo>
  <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.35</wheel_separation>
    <wheel_diameter>0.1</wheel_diameter>
    <max_wheel_torque>20.0</max_wheel_torque>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
    <publish_tf>1</publish_tf>
    <publish_odometry_tf>true</publish_odometry_tf>
  </plugin>
</gazebo>
```

Add this file to your URDF with the following line:
```xml
<xacro:include filename="gazebo_control.xacro"/>
```

### 3. Teleop Keyboard Control
We’ll use the `teleop_twist_keyboard` package to send velocity commands to the robot. Install the package and run the teleop command:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use your keyboard to control the robot. For example:
- Press **I** to move forward.
- Press **,** to move backward.
- Press **J** and **L** to turn left and right.

## Visualizing the Simulation in RViz
To visualize the robot’s movement and odometry in RViz, run RViz in a new terminal:
```bash
rviz2
```
1. Set the fixed frame to `odom`.
2. Add a **RobotModel** display.
3. Add a **TF** display to view transforms between `odom` and `base_link`.

As you control the robot in Gazebo, you should see the robot's odometry update in RViz, allowing you to visualize its movement within the simulated environment.

## Creating a Custom Gazebo World
To make the simulation environment more interesting, create a custom world file for your robot to navigate. Save the world file inside your package’s `worlds` directory:
```xml
<world>
  <!-- Your custom world elements go here -->
</world>
```
Save it as `obstacles.world` and update your launch file to include the world file:
```bash
ros2 launch <your_package> sim.launch.py world:=<path_to_world_file>
```

Now you can navigate your robot around the custom environment you created!

## Troubleshooting Tips
If your robot isn’t moving correctly:
- Ensure your URDF file has proper **inertia** and **friction** values for all links.
- Check that your wheel separation and diameter are set correctly in the control plugin.
- Use `ros2 topic echo /cmd_vel` to verify that velocity commands are being sent to the robot.

## Conclusion
You have successfully driven your virtual robot in a Gazebo simulation, integrated with ROS2. In future tutorials, we will enhance the robot with sensors and start building real hardware. Stay tuned for more!

If you have any questions, feel free to comment or ask for help.

### Next Steps:
- **Watch out for the next tutorial**, where we’ll begin looking at the hardware aspects of the robot, including using a Raspberry Pi as the robot's brain.
- **Prepare for sensor integration** such as LiDAR and camera modules, which we will add to both the simulated and real-world robots.
