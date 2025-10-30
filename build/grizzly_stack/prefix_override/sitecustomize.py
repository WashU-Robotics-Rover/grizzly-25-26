import sys
if sys.prefix == '/opt/anaconda3/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/danielhuinda/robotics/grizzly-25-26/install/grizzly_stack'
