
# Easy SLAM with ROS Using slam_toolbox

In this tutorial, we will explore how to perform SLAM (Simultaneous Localization and Mapping) on our robot using a 2D LiDAR sensor and the `slam_toolbox` package in ROS2. We will cover both simulation and real-world applications. By the end of this guide, you should be able to generate a map of your environment, localize your robot, and save your maps for future use.

---

### Overview of SLAM (Simultaneous Localization and Mapping)

SLAM is a process by which a robot creates a map of its surroundings while simultaneously localizing itself within that map. This is essential when you don't have any GPS or external localization system, as the robot needs to track its movement and surroundings to navigate effectively.

#### Basic Concepts:
- **Localization**: Determining the robot's position on a known map.
- **Mapping**: Creating a map of the environment using the robot's sensors.
- **SLAM**: A process where the robot performs both localization and mapping simultaneously.

---

## Coordinate Frames in ROS2 SLAM

Before diving into implementation, it's important to understand the coordinate frames used in ROS for SLAM:
- **base_link**: The frame attached to the robot’s body.
- **odom**: Represents the odometry frame. It tracks the robot's position based on wheel odometry.
- **map**: The frame representing the global map that the robot creates.
- **base_footprint**: A 2D projection of `base_link`, often used for robots operating in a 2D environment.

During SLAM, the robot will provide transformations between these frames, and the robot's pose is expressed in terms of these frames.

---

## Running SLAM Toolbox in Simulation (Gazebo)

To begin, we will simulate SLAM using the Gazebo simulator. You should have a robot model with a LiDAR sensor configured.

### Launch Simulation and RViz
1. **Launch the robot in Gazebo**:

```bash
ros2 launch my_bot launch_sim.launch.py world:=./src/my_bot/worlds/obstacles.world
```

2. **Start RViz for visualization**:

```bash
rviz2 -d ./src/my_bot/config/main.rviz
```

---

### Start Teleoperation

To manually drive the robot around, use the `teleop_twist_keyboard` node:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

---

### Start SLAM Toolbox

Next, we will launch the `slam_toolbox` using the `online_async` mode, which will allow the robot to generate a live map:

```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=true
```

In `online_async` mode:
- **Online**: SLAM is performed live using current sensor data.
- **Asynchronous**: SLAM toolbox processes data asynchronously without ensuring that every sensor scan is processed.

---

### Saving and Reusing Maps

Once you have created a map, you can save it for future use. Use the following commands to save the map and serialize it for later localization:

1. **Save the map** (for external use, e.g., navigation):

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map_init.yaml -p use_sim_time:=true
```

2. **Serialize the map** (to reuse with `slam_toolbox`):

In RViz, under the **slam_toolbox plugin**:
- Click **Save Map** to store the map.
- Click **Serialize Map** to save it for reuse with `slam_toolbox`.

---

### Localization with the Saved Map

Once the map is saved, you can localize the robot in that map. To do so, launch the SLAM toolbox again, but this time in **localization mode**:

```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=true
```

### AMCL (Adaptive Monte Carlo Localization)

In addition to SLAM Toolbox, you can use the AMCL algorithm for localization:

1. **Run the Map Server**:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map_init.yaml -p use_sim_time:=true
```

2. **Bring Up the Map Server**:

```bash
ros2 run nav2_util lifecycle_bringup map_server
```

3. **Launch AMCL**:

```bash
ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true
```

4. **Bring Up AMCL**:

```bash
ros2 run nav2_util lifecycle_bringup amcl
```

---

## Running SLAM Toolbox on the Physical Robot

Once you have practiced SLAM in simulation, you can run it on your actual robot. The process is almost identical, except that you will now use real sensor data. Be sure to adjust the configuration to use **real-time data** by setting `use_sim_time` to `false`.

### Running SLAM on the Robot:
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/my_bot/config/mapper_params_online_async.yaml use_sim_time:=false
```

---

## Conclusion

SLAM is an essential technique for enabling robots to autonomously navigate and map their environment. Using the SLAM toolbox, we can easily set up SLAM in both simulation and real-world applications. Once we generate a map, we can use it for autonomous navigation in future tasks.

In the next tutorial, we will integrate the `nav2` stack to enable full navigation using the generated map.

---

### Credits

Special thanks to Steve Macenski and the contributors to the ROS2 ecosystem for the development of SLAM Toolbox and the Nav2 stack.

