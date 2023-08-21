import sys
import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Send a positive x value, negative y, negative z,
# positive rotation and no button.
# https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
# Warning: Because of some legacy workaround, z will work between [0-1000]
# where 0 is full reverse, 500 is no output and 1000 is full throttle.
# x,y and r will be between [-1000 and 1000].

master.arducopter_arm()
#buttons = 0 + 0 << 1 + 0 << 2 + 0 << 3 + 0 << 4 + 0 << 5 + 0 << 6 + 0 << 7 + 0 << 8 + 0 << 9 + 1 << 10 + 0 << 11 + 0 << 12 + 0 << 13 + 0 << 14
buttons = 0 << 10
master.mav.manual_control_send(
    master.target_system,
    0,#x(4000 is possible)
    0,#y
    500,#z
    0,#r
    buttons)

time.sleep(3)

master.mav.manual_control_send(
    master.target_system,
    0,#x
    0,#y
    500,#z
    0,#r
    0)

master.arducopter_disarm()
