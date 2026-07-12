import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/monica/repositories/greenhouse-nav/greenhouse_nav/install/greenhouse_nav'
