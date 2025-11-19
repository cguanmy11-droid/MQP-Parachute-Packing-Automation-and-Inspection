import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/my11/interbotix_ws/install/interbotix_perception_modules'
