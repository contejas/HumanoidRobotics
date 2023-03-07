import odrive
import numpy as np
import time


class Motor(object):
    def __init__(self):
        self.odrv0 = odrive.find_any()
        self.velocity = 0
        self.ramprate = 0

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


    def pick_constants(self):
        self.velocity = int(input("Velocity? "))
        self.ramprate = int(input("Ramp Rate/Accel? "))


    def keyboard_loop(self):
        print("Velocity Loop, you control the direction - dont crash it")
        self.pick_constants()
        self.odrv0.axis0.controller.config.input_mode = odrive.utils.InputMode.VEL_RAMP
        self.odrv0.axis0.controller.config.control_mode = odrive.utils.ControlMode.VELOCITY_CONTROL
        self.odrv0.axis0.requested_state = odrive.utils.AxisState.CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.vel_ramp_rate = self.ramprate
        self.odrv0.axis0.controller.input_vel = self.velocity
        while True:
            try:
                cmd = input("CLICK TO SWITCH, STOP to STOP: ")
                if cmd.lower() == "stop":
                    self.stop()
                else:
                    self.velocity = -self.velocity
                    self.odrv0.axis0.controller.input_vel = self.velocity
            except KeyboardInterrupt:
                self.stop()


    def auto_loop(self):
        print("Velocity Loop, will automatically change direction")
        self.pick_constants()
        self.odrv0.axis0.controller.config.input_mode = odrive.utils.InputMode.VEL_RAMP
        self.odrv0.axis0.controller.config.control_mode = odrive.utils.ControlMode.VELOCITY_CONTROL
        self.odrv0.axis0.requested_state = odrive.utils.AxisState.CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.vel_ramp_rate = self.ramprate
        self.odrv0.axis0.controller.input_vel = self.velocity
        sleeptime = 3
        while True:
            try:
                time.sleep(sleeptime)
                self.velocity = -self.velocity
                print(self.velocity)
                self.odrv0.axis0.controller.input_vel = self.velocity
                time.sleep(sleeptime)
            except KeyboardInterrupt:
                self.stop()


    def dump_errors(self):
        odrive.utils.dump_errors(self.odrv0)


    def stop(self):
        self.odrv0.axis0.requested_state = odrive.utils.AxisState.IDLE

motor = Motor()
motor.configurate(calibrate=False)
#motor.dump_errors()
#motor.keyboard_loop()
motor.auto_loop()
#motor.stop()
