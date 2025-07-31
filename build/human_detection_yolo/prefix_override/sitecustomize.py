import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jakgritb/turtlebot3_ws/install/human_detection_yolo'
