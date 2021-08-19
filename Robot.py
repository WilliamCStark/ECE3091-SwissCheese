import time
import numpy as np
import gpiozero

class Robot:
    def __init__(self, wheel_radius, wheel_sep, motor_l, motor_r, rotary_encoder_l, rotary_encoder_r):
        self.x = 0.0 # y-pos (in cm)
        self.y = 0.0 # x-pos (in cm)
        self.th = 0.0 # orientation (in radians)

        self.wl = 0.0 # rotational velocity of the left wheel (in rad/s)
        self.wr = 0.0 # rotational velocity of the right whel (in rad/s)

        self.wheel_radius = wheel_radius # (in cm)
        self.wheel_sep = wheel_sep # (in cm)

        self.motor_l = motor_l # the PWM output device for the left motor
        self.motor_r = motor_r # the PWM output device for the right motor
        self.encoder_l = rotary_encoder_l # the rotary encoder object for the left wheel
        self.encoder_r = rotary_encoder_l # the rotary encoder object for the left wheel

        self.previous_time = time.time()
        self.previous_steps = 0

        # handling the registering of tasks
        # registered movement tasks go into the queue. these are base tasks like moving a distance, or rotating by an angle, and can be composited to achieve any complicated task with the robot
        self.tasks_queue = list()
        # all tasks involve some quantity reaching some target, for example,
        # moving forward for 10 seconds. we must store the current elapsed quantity
        # and target quantity to know when the task has been completed
        self.task_elapsed_quantity = 0
        self.task_elapsed_target = 0

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
        self.wr = self.get_encoder_angular_vel(self.encoder_r, dt) # get left motor angular vel

        v, w = self.base_velocity(self.wl,self.wr) # get the base velocity from wheel rotations

        self.x = self.x + dt*v*np.cos(self.th)
        self.y = self.y + dt*v*np.sin(self.th)
        self.th = self.th + w*dt

        self.previous_time = time.time()

    ################################################################################
    ### Below are the basic drive functions that move the robot in one of it's two
    ### degrees of freedom, either forward motion or rotation. at this level collision
    ### is not considered, as such it is not recommended to use this function directly
    ################################################################################

    # drive_forward(self, min_duty_cycle):
    # Definition: function should drive the robot forward, at a target duty cycle indicated target_duty_cycle
    # will have to provided duty_cycles to each l and r in order to get the robot moving in a straight line
    # may need to take in rotational speed of motor to calculate the duty cycle
    def drive_forward(self, min_duty_cycle):
        pass

    # drive_rotate(self, target_duty_cycle, direction):
    # Definition: will rotate the robot in a specified direction. robot should rotate on the StepsToDistance
    # so may have to control the duty cycles of each to achieve this result. as such
    # target_duty_cycle will be the target duty cycle, but small differences may be required
    # due to differences in the duty cycle products of each motor.
    def drive_rotate(self, target_duty_cycle, direction):
        pass

    ###############################################################################################
    ### Below are base task registering functions. These attempt to complete some task until completion,
    ### or until the robot 'collides' with something.
    ################################################################################################
    # drive_forward_for_time(self, time, target_duty_cycle=1):
    # Defintion: will drive the robot forward in a straight line for the specified amount of time
    def drive_forward_for_time(self, time, target_duty_cycle=1):
        pass
    # drive_forward_for_distance(self, distance, target_duty_cycle=1)
    # Defintion: will drive the robot forward in a straight line for the specified distance
    def drive_forward_for_distance(self, distance, target_duty_cycle=1):
        pass
    # drive_rotate_for_time(self, time, direction, target_duty_cycle=1)
    # Defintion: will rotate the robot in the specified direction for an amount of time
    def drive_rotate_for_time(self, time, direction, target_duty_cycle=1):
        pass
    # drive_rotate_for_angle(self, angle, target_duty_cycle=1)
    # Defintion: will rotate the robot by the specified angle
    def drive_rotate_for_angle(self, angle, target_duty_cycle=1):
        pass
    # drive_rotate_to_angle(self, angle, target_duty_cycle=1)
    # Defintion: will rotate the robot to a specified global angle
    def drive_rotate_to_angle(self, angle, target_duty_cycle=1):
        pass
    # drive_to_point(self, dest_x, dest_y)
    # Defintion: will drive the robot to the destination location, in a straight line
    def drive_to_point(self, dest_x, dest_y):
        pass

    # update function used to drive the robot based on the tasks required of it
    def update(self):
        pass

class Motor:
    def __init__(self, pwm_output, direction_output):
        self.PWM = pwm_output
        self.DIR = direction_output
