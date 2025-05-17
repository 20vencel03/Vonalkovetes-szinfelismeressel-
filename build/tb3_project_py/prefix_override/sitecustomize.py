import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vencel/ros2_ws/src/Vonalkovetes-szinfelismeressel-/install/tb3_project_py'
