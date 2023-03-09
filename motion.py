import odrive
import numpy as np
import time
from inputoutput import valid_int_input


class Motor(object):
    def __init__(self):
        self.odrv0 = odrive.find_any()
        self.velocity = 0
        self.ramprate = 0
        self.position = 0
        self.pos_filter = 0
        self.lower_bound = np.nan
        self.upper_bound = np.nan

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
        # self.odrv0.config.dc_max_negative_current = -0.009999999999
        # self.odrv0.config.max_regen_current = 0.0089999999999
        # self.odrv0.axis0.motor.config.enable_torque_mode_vel_limit = False
        if calibrate:
            print("RUNNING CALIBRATION - DON'T TOUCH")
            self.odrv0.axis0.requested_state = odrive.utils.AxisState.FULL_CALIBRATION_SEQUENCE
            time.sleep(20)
            print("CALIBRATION COMPLETE")

    def pick_vel_constants(self):
        self.velocity = valid_int_input("Velocity? ")
        self.ramprate = valid_int_input("Ramp Rate/Accel? ")

    def pick_pos_constants(self):
        self.pos_filter = valid_int_input("Position Filter? ")

    def keyboard_loop(self):
        print("Velocity Loop, you control the direction - dont crash it")
        self.pick_vel_constants()
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
        self.pick_vel_constants()
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

    def position_loop(self):
        self.calibrate_bounds()
        print("Position Loop, you decide what position in turns/sec it goes to")
        self.pick_pos_constants()
        self.odrv0.axis0.controller.config.input_mode = odrive.utils.InputMode.POS_FILTER
        self.odrv0.axis0.controller.config.control_mode = odrive.utils.ControlMode.POSITION_CONTROL
        self.odrv0.axis0.requested_state = odrive.utils.AxisState.CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.input_filter_bandwidth = self.pos_filter
        self.odrv0.axis0.controller.input_pos = self.position
        while True:
            try:
                self.position = valid_int_input("New Position? ", self.upper_bound, self.lower_bound)
                self.odrv0.axis0.controller.input_pos = self.position
            except KeyboardInterrupt:
                self.stop()

    def calibrate_bounds(self):
        print("This will calibrate the bounds of the machine")
        self.pick_pos_constants()
        self.odrv0.axis0.controller.config.input_mode = odrive.utils.InputMode.POS_FILTER
        self.odrv0.axis0.controller.config.control_mode = odrive.utils.ControlMode.POSITION_CONTROL
        self.odrv0.axis0.requested_state = odrive.utils.AxisState.CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.input_filter_bandwidth = self.pos_filter
        self.odrv0.axis0.controller.input_pos = self.position
        direction = 1
        go = True
        while go:
            cmd = input("If we're at a limit, enter STOP, otherwise print: ")
            if cmd.lower() == "stop":
                if direction == 1:
                    self.upper_bound = self.position
                    print("Sick, going the other way now!")
                    direction *= -1
                elif direction == -1:
                    self.lower_bound = self.position
                    print("Sweet, we're done here!")
                    go = False
            else:
                self.position += 1 * direction
                self.odrv0.axis0.controller.input_pos = self.position
        if self.upper_bound <= self.lower_bound:
            temp = self.upper_bound
            self.upper_bound = self.lower_bound
            self.lower_bound = temp
        print("Upper Bound: " + str(self.upper_bound))
        print("Lower Bound: " + str(self.lower_bound))
        self.stop()

    def dump_errors(self):
        odrive.utils.dump_errors(self.odrv0)

    def stop(self):
        self.odrv0.axis0.requested_state = odrive.utils.AxisState.IDLE


def main():
    motor = Motor()
    motor.configurate(calibrate=True)
    motor.dump_errors()
    motor.position_loop()
    #motor.keyboard_loop()
    #motor.auto_loop()
    #motor.stop()

main()