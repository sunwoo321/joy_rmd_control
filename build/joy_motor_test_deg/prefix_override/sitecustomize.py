import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/s/joy_deg_test_ws/install/joy_motor_test_deg'
