
# ROS2 Navigation and Nav2 Stack Setup

In this guide, we will explore how to set up navigation using the ROS2 Nav2 stack. We'll cover the necessary steps, from setting up your robot in simulation, configuring twist multiplexer, integrating localization, and running the Nav2 stack for autonomous navigation.

### Pre-requisites

- Installed ROS2 Humble or Foxy.
- Installed necessary packages for navigation and twist multiplexer.

```bash
sudo apt install ros-humble-nav2-bringup ros-humble-twist-mux
```

## Overview

In this section, we're setting up the navigation stack using ROS2 `nav2_bringup` and `twist_mux`. This will allow us to control the robot using joystick inputs or autonomous navigation commands.

### Twist Multiplexer (twist_mux)

We'll use `twist_mux` to combine different sources of velocity commands (e.g., joystick, navigation stack) and publish them as a single output to control the robot.

### Configuration for twist_mux

Create a `twist_mux.yaml` file in your `my_bot` configuration directory:

```yaml
topics:
  - topic: cmd_vel
    timeout: 0.5
    priority: 20
    short_desc: "Navigation commands"
  - topic: cmd_vel_joy
    timeout: 0.5
    priority: 100
    short_desc: "Joystick commands"
```

Then run `twist_mux` with the following command:

```bash
ros2 run twist_mux twist_mux --ros-args --params-file ./src/my_bot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped
```

## ROS2 Nav2 Setup

Now, let’s move on to setting up the Nav2 stack. First, we will need to launch the navigation stack. To do this, copy the necessary files into your workspace:

```bash
cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml src/my_bot/config/
cp /opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py src/my_bot/launch/
cp /opt/ros/humble/share/nav2_bringup/launch/localization_launch.py src/my_bot/launch/
```

### Launching Navigation

We’ll now launch the navigation stack with a simulation environment or on the actual robot.

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

To ensure proper localization with a saved map:

```bash
ros2 launch nav2_bringup localization_launch.py map:=./map_init.yaml use_sim_time:=true
```

This launch file will initiate AMCL-based localization with the map provided in `map_init.yaml`. Make sure that the map was saved using either SLAM toolbox or a previous localization run.

To run both localization and navigation together:

```bash
ros2 launch my_bot navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
```

This command configures the navigation stack to use a map for localization while subscribing to updates on the map in a transient local mode, ensuring that navigation operates smoothly.

## Code Structure

Here’s a breakdown of how the structure works:

1. **Twist Mux Configuration**: Allows multiplexing velocity commands from various sources.
2. **Nav2 Launch**: Configures the navigation stack to run alongside localization and SLAM.
3. **Parameters File**: The `nav2_params.yaml` contains all necessary configurations for the Nav2 stack.
4. **Launch Files**: Used to bring up the navigation system and localization.

You can add the following commands to your workflow to automate launching the Nav2 stack and localization:

```bash
ros2 launch my_bot navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
```

## Using Waypoints

Nav2 stack supports waypoint-based navigation. After launching the navigation system, you can set goals manually by clicking in RViz or even define waypoints for the robot to follow.

1. Open RViz, add the `Nav2 Goal` panel.
2. Click on the 2D navigation goal button to send a target position to the robot.

### Advanced Navigation

You can enhance your navigation setup with customized configurations by adjusting parameters in the `nav2_params.yaml` file, including tuning for the robot's dynamics, obstacle inflation, and costmap layers.

## Example Commands

Here is a list of the commands used throughout this guide:

- **Twist Mux for combining command sources**:

  ```bash
  ros2 run twist_mux twist_mux --ros-args --params-file ./src/my_bot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped
  ```

- **Launching Nav2 Navigation**:

  ```bash
  ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
  ```

- **Launching AMCL Localization**:

  ```bash
  ros2 launch nav2_bringup localization_launch.py map:=./map_init.yaml use_sim_time:=true
  ```

- **Copying Nav2 configuration files**:

  ```bash
  cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml src/my_bot/config
  cp /opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py src/my_bot/launch/
  cp /opt/ros/humble/share/nav2_bringup/launch/localization_launch.py src/my_bot/launch/
  ```

## Conclusion

With this setup, your robot should be able to autonomously navigate its environment using a pre-saved map, along with a twist multiplexer to manage joystick and autonomous control inputs.
