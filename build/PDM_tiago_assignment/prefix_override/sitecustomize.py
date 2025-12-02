import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/raaf/PDM_tiago_ws/install/PDM_tiago_assignment'
