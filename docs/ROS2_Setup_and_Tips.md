# ROS 2 Setup and Useful Tips

## 1. Setup Script

When first using ROS 2, one common mistake is trying to find ROS 2 packages or execute nodes without using the ROS 2 setup script. After building a new package, don't forget to run the following setup script in your terminal. Alternatively, as mentioned in the ROS 2 development environment setup tutorial, you can save it to your `~/.bashrc` for convenience:

```bash
source ~/robot_ws/install/local_setup.bash
```

## 2. setup.bash vs local_setup.bash

### 2.1 Underlay and Overlay

Before diving in, let's understand the concepts of underlay and overlay. When you install ROS 2 from binaries (binary installation), all configuration files will be located in the installation folder, e.g., `/opt/ros/foxy/`. If you build from source (source installation), you might use a workspace like `~/ros2_foxy/`. This development environment is referred to as the underlay in ROS.

On the other hand, developers may use a separate workspace, e.g., `~/robot_ws/` for robot development and `~/test_ws` for testing. These development environments are called overlays in ROS.

Overlay development environments depend on the installed ROS packages, making them dependent on the underlay environment. This is why the order of calling and using setup scripts, like `setup.bash`, varies.

### 2.2 Usage of setup.bash and local_setup.bash

Both `setup.bash` and `local_setup.bash` are setup scripts found in all workspaces, whether underlay or overlay, with slightly different purposes.

- **local_setup.bash:** Configures the environment for all packages in the prefix path (e.g., `~/robot_ws/install/`), excluding any upper workspaces.

- **setup.bash:** Includes `local_setup.bash` scripts for all other workspaces provided during the build, meaning it also includes settings from the underlay environment.

If you only have one workspace, there's no difference between using `local_setup.bash` or `setup.bash`. However, with multiple workspaces, they should be used according to their purposes.

For example, to configure the environment to include both your workspace (e.g., `~/robot_ws/install`) and the ROS installation environment (e.g., `/opt/ros/foxy`), use your workspace's `setup.bash` (e.g., `~/robot_ws/install/setup.bash`).

This has the same effect as first sourcing `/opt/ros/foxy/(local_)setup.bash` and then sourcing `~/robot_ws/install/local_setup.bash`.

In short, it's standard practice to first source the underlay environment's `setup.bash`, followed by sourcing your workspace's `local_setup.bash`. Adding these lines to your `.bashrc` makes it easier:

```bash
source /opt/ros/foxy/setup.bash
source ~/robot_ws/install/local_setup.bash
```

## 3. colcon_cd

By using the `colcon_cd` command in your terminal, you can quickly change the shell's current working directory to a package directory. Think of it as the ROS 2 equivalent of ROS 1's `roscd` shell command.

```bash
$ colcon_cd package_name
```

To use this, add the following configuration to your `.bashrc`. If your default workspace is `~/robot_ws`, don't forget to specify `_colcon_cd_root` as shown below:

```bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/robot_ws
```

## 4. ROS_DOMAIN_ID vs Namespace

When using ROS on the same network with others, nodes from different researchers may easily connect and share data. This feature is very useful for multi-robot control and collaboration but can be inconvenient for independent tasks. To prevent this, consider the following three methods:

### 4.1 Use a Separate Physical Network

This is the most straightforward solution: use independent switch hubs and routers to separate your network from others.

### 4.2 Change DDS Domain Using ROS_DOMAIN_ID

ROS 2 uses DDS for multicast communication, allowing you to publish and subscribe to topics in the DDS Global Space. To easily change this DDS Global Space, ROS 2 provides the `ROS_DOMAIN_ID` environment variable. Each RMW (ROS Middleware) references this variable to change domains.

For example, running the following commands in different terminals sets up communication between nodes with the same `ROS_DOMAIN_ID`. By default, `ROS_DOMAIN_ID` values range from 0 to 101, depending on the RMW and operating system.

```bash
$ export ROS_DOMAIN_ID=11
$ ros2 run demo_nodes_cpp talker
$ export ROS_DOMAIN_ID=12
$ ros2 run demo_nodes_cpp listener
$ export ROS_DOMAIN_ID=11
$ ros2 run demo_nodes_cpp listener
```

### 4.3 Change Node and Topic/Service/Action Names Using Namespace

Each ROS 2 node has a unique name, as do the topics, services, actions, and parameters it uses. Changing these unique names using namespaces allows you to create independent network groupings. You can change the namespace by specifying the `ns` variable when launching nodes or by modifying the `node_namespace` field in launch files.

```bash
$ ros2 run turtlesim turtlesim_node
```

Running the above command with an additional namespace argument:

```bash
$ ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/robot1
$ ros2 run turtlesim turtlesim_node --ros-args -r __ns:=/robot2
```

This way, nodes are grouped under the user-specified namespace. However, note that simply changing the namespace only groups the nodes and doesn't hide information from other users.

## 5. Using ROS 1 and 2 on the Same Machine

If you're using both ROS 1 and ROS 2, you can set a `ROS_V` environment variable in your `~/.bashrc` file to switch between them as needed. Here’s an example configuration:

```bash
ROS_V=2 # Select ROS Version 1 or 2

if [ $ROS_V -eq 1 ]; then
  alias cw='cd ~/catkin_ws'
  alias cs='cd ~/catkin_ws/src'
  alias cm='cd ~/catkin_ws && catkin_make'
  source /opt/ros/melodic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  export ROS_MASTER_URI=http://localhost:11311
  export ROS_HOSTNAME=localhost
elif [ $ROS_V -eq 2 ]; then
  source /opt/ros/foxy/setup.bash
  source ~/robot_ws/install/local_setup.bash
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
  source /usr/share/vcstool-completion/vcs.bash
  export ROS_DOMAIN_ID=7
  export ROS_NAMESPACE=robot1
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  alias cw='cd ~/robot_ws'
  alias cs='cd ~/robot_ws/src'
  alias cb='cd ~/robot_ws && colcon build --symlink-install'
  alias cbs='colcon build --symlink-install'
  alias cbp='colcon build --symlink-install --packages-select'
  alias cbu='colcon build --symlink-install --packages-up-to'
  alias rt='ros2 topic list'
  alias re='ros2 topic echo'
  alias rn='ros2 node list'
  alias af='ament_flake8'
  alias ac='ament_cpplint'
  alias testpub='ros2 run demo_nodes_cpp talker'
  alias testsub='ros2 run demo_nodes_cpp listener'
  alias testpubimg='ros2 run image_tools cam2image'
  alias testsubimg='ros2 run image_tools showimage'
fi
```

## 6. Enabling Auto-completion for colcon and vcstool Commands

To use the auto-completion feature for the `colcon` build tool and the `vcstool` version control tool, add the following lines to your `~/.bashrc`:

```bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/vcstool-completion/vcs.bash
```

## 7. Useful Information and Tips

Here are some additional tips and tricks to help you with ROS 2 development:

- Always ensure your setup scripts are sourced properly to avoid issues with finding packages or running nodes.
- Use namespaces and `ROS_DOMAIN_ID` to manage network interactions effectively.
- Use aliases in your `.bashrc` to streamline your workflow in both ROS 1 and ROS 2 environments.
