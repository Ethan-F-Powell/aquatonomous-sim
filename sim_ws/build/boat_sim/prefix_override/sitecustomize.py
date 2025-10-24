import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ethan/aquatonomous-sim/sim_ws/install/boat_sim'
