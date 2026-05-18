import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/badawi/Desktop/auto-pilot/install/auv_gui'
