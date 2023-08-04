"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math

# Find a connected ODrive (this will block until you connect one)
print("Pairing odrives ...")
odrv0 = odrive.find_any(serial_number = "345B367B3330")
odrv1 = odrive.find_any(serial_number = "334A376D3230")
odrv2 = odrive.find_any(serial_number = "3469367C3330")
odrv3 = odrive.find_any(serial_number = "3457367E3330")

def load_config(odrv):
    odrv.config.dc_bus_overvoltage_trip_level = 15
    odrv.config.dc_max_positive_current = 30
    odrv.config.dc_max_negative_current = -4
    odrv.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
    odrv.axis0.config.motor.torque_constant = 0.05513333333333333
    odrv.axis0.config.motor.pole_pairs = 7
    odrv.axis0.config.motor.current_soft_max = 70
    odrv.axis0.config.motor.current_hard_max = 90
    odrv.axis0.config.motor.calibration_current = 10
    odrv.axis0.config.motor.resistance_calib_max_voltage = 2
    odrv.axis0.config.calibration_lockin.current = 10
    odrv.axis0.controller.config.input_mode = InputMode.POS_FILTER
    odrv.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
    odrv.axis0.config.torque_soft_min = -0.7718666666666667
    odrv.axis0.config.torque_soft_max = 0.7718666666666667
    odrv.amt21_encoder_group0.config.event_driven_mode = False
    odrv.amt21_encoder_group0.config.enable = True
    odrv.axis0.config.load_encoder = EncoderId.AMT21_ENCODER0
    odrv.axis0.config.commutation_encoder = EncoderId.AMT21_ENCODER0

## Load Config that seems to work
odrv0.axis0.controller.config.vel_limit = 25
odrv1.axis0.controller.config.vel_limit = 25
odrv2.axis0.controller.config.vel_limit = 25
odrv3.axis0.controller.config.vel_limit = 25
load_config(odrv0)
load_config(odrv1)
load_config(odrv2)
load_config(odrv3)

def setVelo(s1, s2, s3, s4):
    odrv0.axis0.controller.config.vel_limit = s1
    odrv1.axis0.controller.config.vel_limit = s2
    odrv2.axis0.controller.config.vel_limit = s3
    odrv3.axis0.controller.config.vel_limit = s4



print("Bus voltage on Driver 0 is " + str(odrv0.vbus_voltage) + "V")
print("Bus voltage on Driver 1 is " + str(odrv1.vbus_voltage) + "V")
print("Bus voltage on Driver 2 is " + str(odrv2.vbus_voltage) + "V")
print("Bus voltage on Driver 3 is " + str(odrv3.vbus_voltage) + "V")
print("Sucessfully connected and Ready!")

# Calibrate motor and wait for it to finish
print("starting calibration...")
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv1.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv2.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv3.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
while odrv1.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
while odrv2.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
while odrv3.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv2.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv3.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# # Or to change a value, just assign to the property


def setPosi(p0, p1, p2, p3):
    odrv0.axis0.controller.input_pos = p0
    odrv1.axis0.controller.input_pos = p1
    odrv2.axis0.controller.input_pos = p2
    odrv3.axis0.controller.input_pos = p3

while True:
    p0 = input("Enter position for motor 0: ")
    p1 = input("Enter position for motor 1: ")
    p2 = input("Enter position for motor 2: ")
    p3 = input("Enter position for motor 3: ")
    setPosi(p0,p1,p2,p3) #Update Position based on input


# print("Position setpoint is " + str(odrv0.axis0.controller.pos_setpoint))

# # A sine wave to test
# t0 = time.monotonic()
# while True:
#     setpoint0 = 8.0 * math.sin((time.monotonic() - t0)*5)
#     setpoint1 = 8.0 * math.sin((time.monotonic() - t0)*5)
#     setpoint2 = 8.0 * math.sin((time.monotonic() - t0)*5)
#     setpoint3 = 8.0 * math.sin((time.monotonic() - t0)*5)
#     print("goto " + "M0: "  + str(int(setpoint0)) + "M1: "  + str(int(setpoint1)) + "M2: "  + str(int(setpoint2)) + "M3: " + str(int(setpoint3)))
#     odrv0.axis0.controller.input_pos = setpoint0
#     odrv1.axis0.controller.input_pos = setpoint1
#     odrv2.axis0.controller.input_pos = setpoint2
#     odrv3.axis0.controller.input_pos = setpoint3
#     time.sleep(0.01)