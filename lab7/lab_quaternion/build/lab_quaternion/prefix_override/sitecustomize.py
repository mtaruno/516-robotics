import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mtaruno@netid.washington.edu/516-robotics/lab7/lab_quaternion/install/lab_quaternion'
