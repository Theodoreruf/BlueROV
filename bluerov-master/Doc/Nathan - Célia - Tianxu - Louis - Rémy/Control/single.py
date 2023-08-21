# -*- coding: UTF-8 -*-
import time
from pymavlink import mavutil



master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
master.wait_heartbeat()


# Arm 解锁BlueRov2
def arm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)


# Disarm 锁定BlueRov2
def disarm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)



# 设置每个推进器的pwm
def set_motor_pwm(channel, pwm):
    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                                 0, channel, 1, pwm, 1000, 1, 0, 0)






arm()
set_motor_pwm(4, 1100)
time.sleep(0.7)
disarm()
