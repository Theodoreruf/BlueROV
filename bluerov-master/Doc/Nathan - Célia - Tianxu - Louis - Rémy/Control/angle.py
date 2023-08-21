import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()


def angle_data():
    """
    read angle value
    :return: pitch_angle, roll_angle
    """
    theta, phi = 0, 0
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        if msg.get_type() == 'ATTITUDE':
            theta = msg.pitch
            phi = msg.roll
            break
    return theta, phi

master.arducopter_disarm()

pangle,rangle=angle_data()
print('pangle=')
print(pangle)
print('rangle=')
print(rangle)
