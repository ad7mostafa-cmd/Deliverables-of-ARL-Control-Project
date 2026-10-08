import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/adham/Control_project/Control_Project-main/install/bicycle_sim'
