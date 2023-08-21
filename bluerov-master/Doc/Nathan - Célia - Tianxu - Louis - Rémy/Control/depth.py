import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()


def depth_data():
    """
    read value of depth
    :return: depth
    """
    dep = 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'VFR_HUD':
            dep = -msg.alt
            break
    return dep

master.arducopter_disarm()
depth = depth_data()
print('depth=')
print(depth)
