import time
import numpy as np
import gpiozero

class Robot:
    def __init__(self, wheel_radius, wheel_sep, motor_l, motor_r, rotary_encoder_l, rotary_encoder_r):
        self.x = 0.0 # y-pos
        self.y = 0.0 # x-pos
        self.th = 0.0 # orientation

        self.wl = 0.0 # rotational velocity of the left wheel
        self.wr = 0.0 # rotational velocity of the right whel

        self.wheel_radius = wheel_radius
        self.wheel_sep = wheel_sep

        self.motor_l = motor_l # the PWM output device for the left motor
        self.motor_r = motor_r # the PWM output device for the right motor
        self.encoder_l = rotary_encoder_l # the rotary encoder object for the left wheel
        self.encoder_r = rotary_encoder_l # the rotary encoder object for the left wheel

        self.previous_time = time.time()
        self.previous_steps = 0

    def motor_drive(self, motor, duty_cycle, dir):
        motor.PWM.value = duty_cycle
        motor.DIR = dir

    def get_encoder_angular_vel(self, encoder, dt):
        delta_steps = encoder.steps - self.previous_steps
        self.previous_steps = encoder.StepsToDistance
        return delta_steps/dt

    # Veclocity motion model
    def base_velocity(self,wl,wr):

        v = (wl*self.r + wr*self.r)/2.0

        w = (wl - wr)/self.l

        return v, w

    def drive_robot(self, duty_cycle_l, duty_cycle_r, dir_l, dir_r):
        dt = time.time() - self.previous_time
        self.motor_drive(motor_l, duty_cycle_l, dir_l) # drive left motor
        self.motor_drive(motor_l, duty_cycle_l, dir_l) # drive right motor

        self.wl = self.get_encoder_angular_vel(self.encoder_l, dt) # get right motor angular vel
        self.wr = self.get_encoder_angular_vel(self.encoder_r, dt) #get left motor angular vel

        v, w = self.base_velocity(self.wl,self.wr) # get the base velocity from wheel rotations

        self.x = self.x + dt*v*

        self.previous_time = time.time()

class Motor:
    def __init__(self, pwm_output, direction_output):
        self.PWM = pwm_output
        self.DIR = direction_output
