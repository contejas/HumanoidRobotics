import odrive
import numpy as np
import time


class Motor(object):
    def __init__(self):
        self.odrv0 = odrive.find_any()
        self.velocity = 0

    def configurate(self, calibrate=False):
        self.odrv0.clear_errors()
        self.odrv0.axis0.motor.config.current_lim = 15
        self.odrv0.axis0.controller.config.vel_limit = 100
        self.odrv0.config.enable_brake_resistor = True
        self.odrv0.config.brake_resistance = 2
        self.odrv0.axis0.motor.config.pole_pairs = 7
        self.odrv0.axis0.motor.config.torque_constant = 8.27 / 270.0
        self.odrv0.axis0.motor.config.motor_type = 0
        self.odrv0.axis0.encoder.config.cpr = 8192
        #self.odrv0.config.dc_max_negative_current = -0.009999999999
        #self.odrv0.config.max_regen_current = 0.0089999999999
        #self.odrv0.axis0.motor.config.enable_torque_mode_vel_limit = False
        if calibrate:
            print("RUNNING CALIBRATION - DON'T TOUCH")
            self.odrv0.axis0.requested_state = odrive.utils.AxisState.FULL_CALIBRATION_SEQUENCE
            time.sleep(20)
            print("CALIBRATION COMPLETE")


    def closed_loop(self):
        print("CLOSEING LOOP")
        self.odrv0.axis0.controller.config.input_mode = odrive.utils.InputMode.VEL_RAMP
        self.odrv0.axis0.controller.config.control_mode = odrive.utils.ControlMode.VELOCITY_CONTROL
        self.odrv0.axis0.requested_state = odrive.utils.AxisState.CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.vel_ramp_rate = 150  # MAX is 200
        self.velocity = 40
        self.odrv0.axis0.controller.input_vel = self.velocity
        while True:
            cmd = input("CLICK TO SWITCH, STOP to STOP: ")
            if cmd.lower() == "stop":
                self.stop()
            else:
                self.velocity = -self.velocity
                self.odrv0.axis0.controller.input_vel = self.velocity


    def dump_errors(self):
        odrive.utils.dump_errors(self.odrv0)


    def stop(self):
        self.odrv0.axis0.requested_state = odrive.utils.AxisState.IDLE

motor = Motor()
motor.configurate(calibrate=False)
#motor.dump_errors()
#motor.torque_loop()
motor.closed_loop()
#motor.stop()
#print(motor.odrv0.axis0.controller.config.vel_ramp_rate)
#print(motor.odrv0.axis0.controller.input_vel)
#while True:
#    motor.dump_errors()
#print(type(motor.odrv0))
#motor.closed_loop()

