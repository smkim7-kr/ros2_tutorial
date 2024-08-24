### Why ROS2?
1. Development Time Reduction - open=sourced, editing flexibility
2. Supports for Industry Use - ROS1 is mainly for academic usgae
3. Support Multi-Platform - Windows, Linux, MacOS...
4. Support diverse domains - indoor/outdoor etc.
5. Flexibility in choosing vendor - abstraction on different applications and networking
6. Communication Protocaol Standards - ROS2 uses DDS, ROS1 uses TCP/UDP
7. Open-Source - Apache 2.0 license
8. Global Community - Global Support
9. Portability from ROS1 to ROS2 - ROS2 supports bridge from ROS1

### ROS1 vs. ROS2
| Features       | ROS1        | ROS2 |
|------------|------------|------------|
| Platform  | Linux         | Multi-Platform   |
| Real-time control | X         | O     |
| Network environment  | Stable network required         | Works on unstable network    |
| Robot | Single         | Multi     |
| Embedded-Systems | X (rosserial)         | O     |
| Purpose | Academic         | Industry & academic     |

| Features               | ROS 1                                       | ROS 2                                                            |
|------------------------|---------------------------------------------|------------------------------------------------------------------|
| **Platforms**          | Linux, macOS                                | Linux, macOS, Windows                                            |
| **Real-time**          | External frameworks like OROCOS             | Real-time nodes when using a proper RTOS with carefully written user code |
| **Security**           | SROS                                        | SROS 2, DDS-Security, Robotic Systems Threat Model               |
| **Communication**      | XMLRPC + TCPROS                             | DDS (RTPS)                                                       |
| **Middleware interface**| -                                           | rmw                                                              |
| **Node manager(discovery)**| ROS Master                             | No, use DDS’s dynamic discovery                                  |
| **Languages**          | C++03, Python 2.7                           | C++14 (C++17), Python 3.5+                                       |
| **Client library**     | roscpp, rospy, rosjava, rosnodejs, and more | rclcpp, rclpy, rcljava, rcljs, and more                          |
| **Build system**       | rosbuild → catkin (Cmake)                   | Ament (Cmake), Python setuptools (Full support)                  |
| **Build tool**         | catkin_make, catkin_tools                   | colcon                                                           |

### Security of ROS2
ROS1 had issues in secuirty - Failure of master node can kill all running nodes
ROS2 adapts indsutry standard DDS (Data Distribution Service) - protocol standards, "rules"

### Communication of ROS2
ROS1 used own-created TCPROS communcation library
ROS2 uses industry-standard RTPS - supports real-time comm, QoS(Quality of Service)

### Middleware interfacce of ROS2
Fast DDS (default), Cycleone DDS etc.

### Dynamic Discovery of ROS2
ROS1's roscore -> ROS2's automatically donw by DDS

### Build System of ROS2
ROS1's catkin -> ROS2's oolcon  
can work on multiple independent workspaces  
Remove dependencies by independent build  
No devel space - better for package management

### Version Control system of ROS2
ROS2 integrate VCS by vcstool (*.repos)

### Client library of ROS2
Supports diverse languages - rclcpp(C++14), rclpy(Python3)..

### Life cycle of ROS2
Node state (init, run, kill...) can be controlleed by callback in ROS2

### Multiple nodes of ROS2
ROS2 can run multiple nodes in single processor  
include in RCL - within process, communication overhead can be ignored

### Threading model of ROS2
ROS1 only supports single/multi threading
ROS2 supports more diverse threading systems

### Messages of ROS2
Topic, Service, Action + ROS2 has IDL (Interface Description Language)

### roslaunch of ROS2
XML + ROS2 support python modules for roslaunch

### Embedded Systems of ROS2
rosserial of ROS1 -> serial/bluetooth/wifi communication, RTOS support in ROS2

