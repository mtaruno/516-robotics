import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mtaruno@netid.washington.edu/516-robotics/lab4/nav_timer_pkg/install/nav_timer_pkg'
