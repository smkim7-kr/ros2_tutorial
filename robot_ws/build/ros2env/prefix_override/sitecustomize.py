import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/smkim/ros2_tutorials/robot_ws/install/ros2env'
