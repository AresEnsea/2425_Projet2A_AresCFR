import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/carpe-bleue/ros2_ws/install/lidar_vl53l1x_processing'
