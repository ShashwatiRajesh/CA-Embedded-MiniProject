import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amadorjosephg/CA-Embedded-MiniProject/comet_bot_ws/install/lunabot_control'
