import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amadorjosephg/comet_bot_ws/src/install/lunabot_control'
