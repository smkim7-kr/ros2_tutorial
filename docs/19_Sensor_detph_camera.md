
# How to get Your Robot to See in 3D! (Depth Cameras in ROS)

## Introduction

Depth cameras allow robots to see the world in 3D. They capture depth information for each pixel, which can be used for object detection, navigation, and more. In this tutorial, we will explore the following:

1. **What depth cameras are** and how they work.
2. **How to use depth cameras in ROS.**
3. **Simulating a depth camera in Gazebo.**
4. **Connecting to a real depth camera for your robot.**

This guide assumes you've already watched the previous tutorial on using regular cameras in ROS. If not, check it out before proceeding. By the end, you’ll know how to integrate depth cameras into your robotics project.

---

## 1. What are Depth Cameras?

Depth cameras capture not just color or light intensity but the distance (depth) to each object in the scene. Unlike regular cameras that output 2D images, depth cameras provide a third dimension.

### Types of Depth Cameras

Depth cameras work using various techniques, with the three most common being:

1. **Structured Light**: This technology projects a known pattern (usually infrared) onto the environment and captures the distortion of the pattern to infer depth. The original Kinect (Xbox 360) used this method.

2. **Time-of-Flight**: Here, the camera sends pulses of light and measures how long it takes for the light to return. This is similar to LIDAR but uses light instead of laser. The Kinect v2 (Xbox One) utilized this.

3. **Stereo Vision**: Mimicking how our eyes work, stereo depth cameras have two or more lenses. By comparing the images from the two lenses, the system calculates depth. The Oak-D light camera uses this method, which we’ll use later in this tutorial.

### How Depth Cameras Work

- **RGB-D Cameras**: These cameras return two types of data: RGB (color) and Depth (distance). Together, these form an RGB-D image, where each pixel has color information and a corresponding depth value.
- **Calibration**: Depth cameras require precise calibration to ensure that the depth data corresponds accurately to the real world. Misaligned sensors can lead to inaccurate point clouds.
  
Depth cameras are becoming more prevalent due to their applications in **SLAM (Simultaneous Localization and Mapping)**, **object detection**, and **navigation**.

---

## 2. Using Depth Cameras in ROS

### ROS Depth Camera Basics

Depth cameras in ROS are treated similarly to regular cameras. The primary difference is in the data format:

- **Depth Image Data**: Depth images can be stored in two formats:
  - 32-bit floating points: Each pixel represents the depth in meters.
  - 16-bit unsigned integers: Each pixel represents depth in millimeters.

ROS uses these formats to represent depth values across the entire image. Typically, a depth image is accompanied by a camera info topic, which provides metadata about the camera's calibration and lens distortion.

**Example ROS Topics**:
- `/camera/depth/image_raw`: The raw depth image.
- `/camera/depth/camera_info`: The camera's calibration data.
  
#### Depth Image Processing

ROS provides the **depth_image_proc** package, which contains tools to process depth images. This package can:
- Convert depth images between formats (e.g., 16-bit to 32-bit).
- Generate point clouds from depth images.
- Register RGB and depth images to create aligned data.

### Visualization: Normalizing Depth Images

To view depth images in ROS, normalization is often required. This ensures the depth range is mapped to pixel intensity, allowing you to see the differences between near and far objects. ROS visualizes depth images by making closer objects darker and further objects lighter.

---

## 3. Simulating a Depth Camera in Gazebo

Simulating sensors is a great way to test without real hardware. Let's add a depth camera to our robot in Gazebo.

### Step-by-Step Setup

1. **Start with the Camera URDF**: We'll build on the work from the last tutorial by copying the existing `camera.xacro` file and converting it for a depth camera.

```bash
cp camera.xacro depth_camera.xacro
```

2. **Edit the Xacro File**:
   - Change the sensor type from `camera` to `depth`.
   - Adjust parameters like field of view and depth clipping.

```xml
<sensor type="depth" name="depth_camera">
    <pose>0 0 0 0 0 0</pose>
    <update_rate>30</update_rate>
    <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
        </image>
        <clip>
            <near>0.1</near>
            <far>10</far>
        </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <always_on>true</always_on>
        <update_rate>30</update_rate>
    </plugin>
</sensor>
```

3. **Modify the Gazebo Launch File**:
   - Update the launch file to include this new depth camera. 
   - Use a proper namespace for the depth camera topics.

```xml
<include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="world_name" value="$(find my_robot)/worlds/empty.world"/>
</include>
```

4. **Run the Simulation**:
   Build the package and launch the simulation:

```bash
colcon build
source install/setup.bash
ros2 launch my_robot gazebo.launch.py
```

### Visualizing in RViz

To visualize the depth data:

1. Open RViz and add a **PointCloud2** display.
2. Select the depth camera's point cloud topic (`/camera/depth/points`).
3. You should see the point cloud representing the environment in 3D.

---

## 4. Connecting a Real Depth Camera

### Using the Oak-D Lite Depth Camera

For this demo, we'll use the **Oak-D Lite** camera, which offers stereo depth sensing. The Oak-D Lite uses the **Luxonis DepthAI platform**, allowing you to integrate depth data seamlessly into ROS.

#### Installing DepthAI and ROS Drivers

First, clone the necessary repositories:

```bash
git clone https://github.com/luxonis/depthai-ros.git
cd depthai-ros
colcon build
```

After building the workspace, source it:

```bash
source install/setup.bash
```

#### Running the Depth Camera Node

Launch the Oak-D Lite with the following ROS 2 launch command:

```bash
ros2 launch depthai_examples stereo.launch.py camera_model:=oak_d_lite
```

This command will start publishing both RGB and depth data. You can view it in RViz using the `/camera/depth/points` and `/camera/rgb/image_raw` topics.

### Powering Depth Cameras

Depth cameras often require more power than typical sensors. Ensure that your power supply is sufficient, especially if using a Raspberry Pi or similar low-power computer. Under-voltage can cause connectivity and performance issues.

---

## Conclusion

You’ve now successfully learned how to integrate depth cameras into your robot, both in simulation (Gazebo) and using real hardware (Oak-D Lite). Depth cameras are incredibly powerful tools for autonomous navigation, object detection, and 3D mapping, and they can significantly enhance your robot's perception capabilities.

In the next tutorials, we’ll dive into more advanced topics like sensor fusion, ROS 2 control, and SLAM. Stay tuned!
