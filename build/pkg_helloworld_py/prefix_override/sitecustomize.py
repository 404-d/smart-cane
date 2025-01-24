import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rasp/桌面/smart-cane/install/pkg_helloworld_py'
