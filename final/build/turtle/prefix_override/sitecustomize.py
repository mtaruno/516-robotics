import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mtaruno@netid.washington.edu/516-robotics/final/install/turtle'
