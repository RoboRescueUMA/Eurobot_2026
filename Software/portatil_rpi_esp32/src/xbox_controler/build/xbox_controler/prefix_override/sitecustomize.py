import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/axarbot/xbox_controller_rpi/xbox_controler/install/xbox_controler'
