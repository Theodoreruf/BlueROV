#            ##              ##             ##
#          ######          ######         ######
#          ######          ######         ######
#           ####            ####           ####
#           ####            ####           ####
#           ####            ####           ####
#           ####            ####           ####
#            ##              ##             ##
#            ##              ##             ##
#            
#            ##              ##             ##
#           ####            ####           ####
#            ##              ##             ##

#Auteur Tianxu LI    tianxu.li@etu.umontpellier.fr

# Ne modifiez pas ce fichier, c'est un script qui peut contrôler bluerov (avec succès)
# S'il y a des questions plus tard, ce sera un point d'archive

import sys
import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()


#ARM BLUE ROV
def arm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)


#DISARM BLUE ROV
def disarm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)


# BUTTONS
#buttons = 0 + 0 << 1 + 0 << 2 + 0 << 3 + 0 << 4 + 0 << 5 + 0 << 6 + 0 << 7 + 0 << 8 + 1 << 9 + 0 << 10 + 0 << 11 + 0 << 12 + 0 << 13 + 0 << 14
buttons=0

#CONTROL FUNCTION
# X : avant et apres( x>0 = aller avancer, x<0 = reculer )
# Y : translation(y>0 droit, 
# Z : monter et descendre( 500 = stable,z>500 = monter, z<500 = descendre)
# R : tourner(r>0,gauch

def ctrl(x,y,z,r,buttons):
    master.mav.manual_control_send(
        master.target_system,
        x,
        y,
        z,
        r,
        buttons)



# Choose a mode
mode = 'MANUAL'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# Get mode ID
mode_id = master.mode_mapping()[mode]
# Set new mode
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)
while True:
    # Wait for ACK command
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()

    # Check if command in the same in `set_mode`
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue

    # Print the ACK result !
    print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
    break



arm()
print('BLUEROV ARMED !!!')
#COMMAND ICI


ctrl(0,0,500,0,1<<11)
time.sleep(3)
buttons=0
ctrl(0,0,500,0,buttons)
time.sleep(2)


"""
ctrl(200,0,500,0,buttons)
time.sleep(2)
ctrl(-200,0,500,0,buttons)
time.sleep(2)

ctrl(0,0,700,0,buttons)
time.sleep(2)
ctrl(0,0,300,0,buttons)
time.sleep(2)

ctrl(0,0,500,200,buttons)
time.sleep(2)
ctrl(0,0,500,-200,buttons)
time.sleep(2)


buttons=1<<9
ctrl(0,0,500,0,buttons)
time.sleep(1)
buttons=0
ctrl(0,0,500,0,buttons)
time.sleep(2)
buttons=1<<10
ctrl(0,0,500,0,buttons)
time.sleep(2)
buttons=0
ctrl(0,0,500,0,buttons)
time.sleep(2)
buttons=1<<7
ctrl(0,0,500,0,buttons)
time.sleep(2)
"""

#STOP BLUE ROV
master.mav.manual_control_send(
    master.target_system,
    0,#x
    0,#y
    500,#z
    0,#r
    0)

disarm()
print('finished')

