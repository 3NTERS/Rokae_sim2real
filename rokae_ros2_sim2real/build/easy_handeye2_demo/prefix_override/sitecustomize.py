import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rokae/workspace/rokae_ros2_sim2real/install/easy_handeye2_demo'
