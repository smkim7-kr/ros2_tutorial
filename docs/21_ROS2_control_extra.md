
# ROS2 Control Extra Bits

This document is an extended guide to complement the **ROS2 Control** video tutorial. If you are following along with the mobile robot build project, the following configurations and fixes will help ensure the robot operates correctly with **ros2_control**.

## Introduction

In the main **ROS2 Control** video tutorial, we covered the basic setup and concepts of the **ros2_control** framework, which helps manage robot hardware interfaces and controllers. However, there are a few extra configurations and bug fixes that are necessary to ensure **ros2_control** runs smoothly on a real robot, and this guide will go over these.

**Note:** For the demonstrations in this video, I used a controller to drive the robot, which provides smoother control during the setup process. You can still follow along using **teleop_twist_keyboard**.

---

## Part 1: Upgrading the URDF to Match the Robot Build

The first task is upgrading your URDF (Unified Robot Description Format) to match the actual robot you’ve built. This involves adjusting the parameters such as joint positions, collision geometries, and other robot characteristics.

You can define the robot’s physical characteristics, such as wheel radius, joint positions, and more. Here’s a simple upgrade process:

```xml
<robot name="mobile_robot">
  <link name="base_link">
    <visual>
      <!-- Visualization of the robot chassis -->
    </visual>
    <collision>
      <!-- Collision properties -->
    </collision>
  </link>

  <joint name="wheel_joint_left" type="continuous">
    <!-- Joint definition for the left wheel -->
  </joint>
  <joint name="wheel_joint_right" type="continuous">
    <!-- Joint definition for the right wheel -->
  </joint>
</robot>
```

Adjust these fields according to your robot's actual configuration.

## Part 2: Fixing the Choppy Movement in Gazebo

When you run the simulation, you may notice that the robot movement appears choppy. This issue is caused by the slow communication rate between Gazebo and ROS, as Gazebo controls the simulation clock and pulses the simulation at a low rate (10 Hz). To fix this:

1. **Create a new parameters file**: Define the publish rate in Gazebo so that it communicates at a higher rate with ROS.
   
    Create a file called `gazebo_params.yaml` with the following content:

    ```yaml
    gazebo:
      ros__parameters:
        publish_rate: 400.0
    ```

2. **Update the launch file**: Add the path to this new parameters file in your simulation launch script:

    ```python
    gazebo_params_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'config', 'gazebo_params.yaml')

    gazebo_args = '--ros-args --params-file ' + gazebo_params_file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
        launch_arguments={'extra_gazebo_args': gazebo_args}.items(),
    )
    ```

By setting the publish rate to 400 Hz, you’ll ensure smoother interaction between the ROS nodes and Gazebo.

---

## Part 3: Reducing Wheel Drift

When running the simulation, you might notice that your robot's wheels drift slightly, leading to misalignment between the robot's actual position and what is shown in **RViz**. This can be caused by the cylinder shape of the wheels, which creates friction over a wider area.

### Solution

To reduce this, you can change the collision geometry of the wheels from a cylinder to a sphere, as spheres will provide a more idealized point of contact with the ground.

In your URDF file, modify the collision section for the wheels:

```xml
<link name="wheel_left">
  <collision>
    <geometry>
      <sphere radius="0.05" />
    </geometry>
  </collision>
</link>
```

This reduces the contact area and improves the accuracy of the robot's movement.

---

## Part 4: Addressing the Position Jumping in RViz

Another issue you might encounter is the "jumping" effect of obstacles in **RViz** as the robot moves. This could interfere with SLAM or navigation algorithms. This issue is often caused by incorrect timing in the controller or wheel odometry setup.

### Possible Fix

Unfortunately, this issue is still under investigation, but a potential solution is to adjust the simulation's parameters and verify that the timing for odometry data is correct. Lowering the speed of the robot might reduce the severity of this issue, though it doesn’t solve it entirely.

---

## Part 5: Switching Between ros2_control and Gazebo Plugins

In some cases, you may want to switch between using **ros2_control** and the **Gazebo** built-in differential drive plugin. This is especially useful if you're facing issues with the **ros2_control** setup.

### Adding a Xacro Argument

To toggle between the two systems, modify the **Xacro** file with an argument to choose between **ros2_control** and the **Gazebo** plugin. Use the following approach in your URDF file:

```xml
<xacro:arg name="use_ros2_control" default="true" />

<xacro:if value="${use_ros2_control}">
  <ros2_control>
    <!-- ros2_control configuration here -->
  </ros2_control>
</xacro:if>
<xacro:unless value="${use_ros2_control}">
  <gazebo>
    <!-- Gazebo differential drive plugin here -->
  </gazebo>
</xacro:unless>
```

This **Xacro** conditional will allow you to toggle between the two systems. By default, it uses **ros2_control**, but you can change the launch argument to switch to **Gazebo** control.

### Update the Launch File

Finally, update your **launch** file to pass the `use_ros2_control` argument to the **Xacro** processor:

```python
use_ros2_control = LaunchConfiguration('use_ros2_control')

robot_description = Command(
    [
        'xacro ', os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro'), 
        ' use_ros2_control:=', use_ros2_control
    ]
)
```

This change allows you to easily swap between the two systems.

---

## Conclusion

These fixes and improvements should help you prepare your simulation and actual robot for **ros2_control**. This extra bit of configuration will ensure smoother operation and allow you to easily toggle between the different control systems.

If you encounter any issues or have suggestions, feel free to join the conversation on the community forums.

Thanks again for following along, and special thanks to my Patreon supporters for making these tutorials possible! Stay tuned for more robot-building tutorials.
