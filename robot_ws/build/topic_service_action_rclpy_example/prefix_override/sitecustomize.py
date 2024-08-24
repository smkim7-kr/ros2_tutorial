import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/smkim/ros2_tutorials/robot_ws/install/topic_service_action_rclpy_example'
