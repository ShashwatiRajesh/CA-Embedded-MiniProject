import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amadorjosephg/CA-Embedded-MiniProject/cometbot_ws/src/install/cometbot_control'
