# ROS Message Communication

Before diving into ROS 2 programming, it's essential to understand some key terms and the concept of message communication, which is critical in both ROS 1 and ROS 2.

## Key Concepts:
- **Nodes:** The smallest executable units in ROS, representing individual programs.
- **Packages:** Collections of nodes and necessary files grouped together.
- **Message Communication:** Nodes exchange data using messages, which can be simple data types or structured data. The communication methods are categorized into:
  - **Topics**
  - **Services**
  - **Actions**
  - **Parameters**

## ROS 2 and DDS

ROS 2 uses DDS (Data Distribution Service) for its message communication, replacing the TCPROS protocol used in ROS 1. DDS is a standardized middleware protocol providing reliable real-time data exchange, crucial for industrial applications.

### Advantages of DDS in ROS 2:
- **Data-centric communication:** Ensures efficient and reliable data exchange.
- **Dynamic Discovery:** Nodes can discover each other without a central ROS Master.
- **Scalable Architecture:** Suitable for both small devices and large systems.
- **Interoperability:** DDS-compliant products from different vendors can work together seamlessly.
- **Quality of Service (QoS):** Allows fine-tuning of communication settings for reliability, performance, and security.

## DDS Overview

DDS (Data Distribution Service) is a middleware protocol designed for data-centric connectivity, providing low-latency, reliable data communication, and scalability.

### Features:
- **Industry Standards**
- **OS Independent**
- **Language Independent**
- **Transport on UDP/IP**
- **Dynamic Discovery**
- **Quality of Service (QoS)**
- **Security**

## Using ROS with DDS

### Running Basic Nodes
You can run a publisher node and a subscriber node in separate terminals. The nodes will use the default ROS 2 middleware (RMW), `rmw_fastrtps_cpp`.

\`\`\`bash
$ ros2 run demo_nodes_cpp listener
$ ros2 run demo_nodes_cpp talker
$ rqt_graph
\`\`\`

### Changing RMW
To switch the RMW implementation, use the `RMW_IMPLEMENTATION` environment variable.

\`\`\`bash
$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
$ ros2 run demo_nodes_cpp listener
$ ros2 run demo_nodes_cpp talker
\`\`\`

### Domain Configuration
Adjust `ROS_DOMAIN_ID` to isolate communication between nodes on different domains.

\`\`\`bash
$ export ROS_DOMAIN_ID=11
$ ros2 run demo_nodes_cpp talker
$ export ROS_DOMAIN_ID=12
$ ros2 run demo_nodes_cpp listener
\`\`\`

### QoS Testing
Test the reliability of data transmission by simulating data loss and observing how the system behaves under different QoS settings.

\`\`\`bash
$ sudo tc qdisc add dev lo root netem loss 10%
$ ros2 run demo_nodes_cpp listener
$ ros2 run demo_nodes_cpp talker
\`\`\`
"""

# Save the content to a .md file
file_path = "/mnt/data/ros_message_communication_summary.md"
with open(file_path, "w") as file:
    file.write(markdown_content)

file_path
