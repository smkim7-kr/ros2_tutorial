from launch import LaunchDescription # generic launch
from launch_ros.actions import Node # ROS2-specific launch
from launch.actions import DeclareLaunchArgument # for declaration of launch arguments
from launch.substitutions import LaunchConfiguration # retrieves value of launch argument


def generate_launch_description():
    return LaunchDescription([
        # run turtlesim simulation
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='tf2_cpp_',
            executable='turtle_tf2_broadcaster',
            name='broadcaster1',
            parameters=[ # node publish transformations for turtle1
                {'turtlename': 'turtle1'} # turtlename set to turtle1 (default: turtle)
            ]
        ),
        DeclareLaunchArgument( # declares launch argument
            'target_frame', default_value='turtle1',
            description='Target frame name.'
        ),
        Node( # launch tf2 broaadcaster of turtle2
            package='tf2_cpp_',
            executable='turtle_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node( # run tf2 listener node
            package='tf2_cpp_',
            executable='turtle_tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')} # listen turtle1 frame
            ]
        ),
    ])