import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ar0ck/TE3003B_Retos/install/puzzlebot_sim'
