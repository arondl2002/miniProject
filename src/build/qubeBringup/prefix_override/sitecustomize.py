import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aron-larsen/miniProject_ws/src/install/qubeBringup'
