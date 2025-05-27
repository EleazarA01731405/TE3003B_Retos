import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ar0ck/ros2_ws_t3003/src/puzzlebot_gazebo/install/puzzlebot_gazebo'
