# -*- coding: utf-8 -*-

# Ce programme arme le robot et permet de le commander en simulant
# des commandes joystick RC

import time
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Create a function to send RC values
# More information about Joystick channels
# here: https://www.ardusub.com/operators-manual/rc-input-and-output.html#rc-inputs
def set_rc_channel_pwm(channel_id, pwm=1500):
    """ Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.


# Les fonctions suivantes permettent de contrôle les mouvements du ROV
# Pitch, Yaw, Roll, Throttle, Forward & Lateral
# La vitesse speed_pct est un pourcentage positif ou négatif [-100 ; 100]

def move_pitch(speed_pct):
    # La commande PWM va de 1100us à 1900us
    # Quand speed_pct vaut 0%, aucune action ne doit être envoyée
    set_rc_channel_pwm(1, 1500+4*speed_pct)

def move_roll(speed_pct):
    set_rc_channel_pwm(2, 1500+4*speed_pct)

def move_throttle(speed_pct):
    set_rc_channel_pwm(3, 1500+4*speed_pct)

def move_yaw(speed_pct):
    set_rc_channel_pwm(4, 1500+4*speed_pct)

def move_forward(speed_pct):
    set_rc_channel_pwm(5, 1500+4*speed_pct)

def move_lateral(speed_pct):
    set_rc_channel_pwm(6, 1500+4*speed_pct)

### ENVOI DE COMMANDES

# Armage du robot (activation)
master.arducopter_arm()

# Envoi d'une commande
# La commande doit être envoyée fréquemment, d'où la boucle while

while 1:
    move_forward(40) # Commande avant 40% de la vitesse max
    # Pour reculer, mettre une valeur négative (ex : -50%)
