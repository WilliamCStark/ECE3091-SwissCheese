import time
import numpy as np
import gpiozero
import queue

class BaseRobot:
    def __init__(self, wheel_radius, wheel_sep, motor_l, motor_r, rotary_encoder_l, rotary_encoder_r, pipe=None):
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
        self.encoder_r = rotary_encoder_r # the rotary encoder object for the right wheel
        self.gear_ratio = 32 # need to have a different gear ratio for each wheel

        self.previous_steps_l = rotary_encoder_l.steps
        self.previous_steps_r = rotary_encoder_r.steps
        self.dt = 0.01 # the time to sleep after applying a motor movement

        self.e_sum_l = 0
        self.e_sum_r = 0

        self.pipe = pipe # the queue to put pertinent robot variables to be delivered to the main thread from an adjacent thread


    def motor_drive(self, motor, duty_cycle, dir):
        #print("in here")
        #print(motor.PWM)
        motor.PWM.value = duty_cycle
        motor.DIR = dir

    def get_encoder_angular_vel(self, encoder, dt, previous_steps):
        # might need to change to accomodate gear ratio
        delta_steps = encoder.steps - previous_steps
        previous_steps = encoder.steps/32 * 2*np.pi # convert steps value to revs/s, then to rad/s
        delta_rots = delta_steps / self.gear_ratio # convert to rad/s for the wheel itself
        return (delta_rots/dt, previous_steps)

    # Veclocity motion model
    def base_velocity(self):
        v = (self.wl*self.wheel_radius + self.wr*self.wheel_radius)/2.0
        w = (self.wl - self.wr)/self.wheel_sep
        return v, w

    # At the end of a given time step, update the 'pose', essentially update the
    # internally stored position of the robot.
    def pose_update(self, duty_cycle_l, duty_cycle_r, dir_l, dir_r):
        dt = self.dt
        self.motor_drive(self.motor_l, duty_cycle_l, dir_l) # drive left motor
        self.motor_drive(self.motor_r, duty_cycle_r, dir_r) # drive right motor

        self.wl, self.previous_steps_l = self.get_encoder_angular_vel(self.encoder_l, dt, self.previous_steps_l) # get right motor angular vel
        self.wr, self.previous_steps_r = self.get_encoder_angular_vel(self.encoder_r, dt, self.previous_steps_r) # get left motor angular vel

        v, w = self.base_velocity() # get the base velocity from wheel rotations

        self.x = self.x + dt*v*np.cos(self.th)
        self.y = self.y + dt*v*np.sin(self.th)
        self.th = self.th + w*dt

        self.previous_time = time.time()

    ################################################################################
    ### Below is the basic drive function. We specify a desired velocity and speed of
    ### rotation.
    ################################################################################

    # drives the robot with the desired velocity and angular velocity, and sleeps for a time step
    def drive(self, v_desired, w_desired):
        wl_desired = v_desired/self.wheel_radius + self.wheel_sep*w_desired/2
        wr_desired = v_desired/self.wheel_radius - self.wheel_sep*w_desired/2

        duty_cycle_l,self.e_sum_l = self.p_control(wl_desired,self.wl,self.e_sum_l)
        duty_cycle_r,self.e_sum_r = self.p_control(wr_desired,self.wr,self.e_sum_r)

        # call pose update with the duty cycle. we reparameterise the duty cycle from -1 to 1 into a 0 to 1 with a single
        # flag for direction.
        self.pose_update(abs(duty_cycle_l), abs(duty_cycle_r), -(abs(duty_cycle_l)/duty_cycle_l-1)/2,-(abs(duty_cycle_r)/duty_cycle_r-1)/2)
        # Push the change to the robot in this thread to the queue so the main thread may update it's representation
        if self.pipe is not None:
            self.push_to_pipe()
            self.check_death()
        time.sleep(self.dt) # sleep after each drive call so we only drive the robot in increments

    # utility function for the drive function, calculates required duty cycle for
    # a desired step velocity and minimizes accumulated error.
    def p_control(self,w_desired,w_measured,e_sum):
        kp = 0.1
        ki = 0.01

        duty_cycle = min(max(-1,kp*(w_desired-w_measured) + ki*e_sum),1)

        e_sum = e_sum + (w_desired-w_measured)

        return duty_cycle, e_sum

    # immediately arrest the motion of the robot
    def stop(self):
        self.motor_l.PWM.value = 0
        self.motor_r.PWM.value = 0

    def check_death(self):
        if self.pipe.poll():
            # if the main thread ever communicates with us, it is to tell us we are about to die
            self.stop() # make sure to cease all motion now we are dead
            self.pipe.close() # close the pipe, so the main thread knows we have finished up
            while True:
                pass

    def push_to_pipe(self):
        msg = [self.x, self.y, self.th] # the only variables we want the thread robot to keep
        self.pipe.send(msg)


class Robot (BaseRobot):
    def __init__(self, wheel_radius, wheel_sep, motor_l, motor_r, rotary_encoder_l, rotary_encoder_r, q=None):
        super().__init__(wheel_radius, wheel_sep, motor_l, motor_r, rotary_encoder_l, rotary_encoder_r, q)
    ###############################################################################################
    ### Below are functions that complete some basic movement of the robot
    ################################################################################################
    # drive_forward_for_time(self, time, target_duty_cycle=1):
    # Defintion: will drive the robot forward in a straight line for the specified amount of time
    def drive_forward_for_time(self, t, v_desired):
        start_time = time.time()
        while (time.time() - start_time) < t:
            self.drive(v_desired, 0)
    # drive_forward_for_distance(self, distance, target_duty_cycle=1)
    # Defintion: will drive the robot forward in a straight line for the specified distance
    def drive_forward_for_distance(self, distance, v_desired):
        current_distance = 0
        while current_distance < distance:
            self.drive(v_desired, 0)
            #current_distance += self.base_velocity()[0]*self.dt
            current_distance += v_desired*self.dt #TEST EDIT
    # drive_rotate_for_time(self, time, direction, target_duty_cycle=1)
    # Defintion: will rotate the robot in the specified direction for an amount of time
    def drive_rotate_for_time(self, t, w_desired):
        start_time = time.time()
        while (time.time() - start_time) < t:
            self.drive(0, w_desired)
    # drive_rotate_for_angle(self, angle, target_duty_cycle=1)
    # Defintion: will rotate the robot by the specified angle
    def drive_rotate_for_angle(self, angle, w_desired):
        current_angle = 0
        while current_angle < angle:
            self.drive(0, w_desired)
            #current_angle += self.base_velocity()[1]*self.dt
            current_angle += w_desired*self.dt #TEST EDIT
    # drive_rotate_to_angle(self, angle, target_duty_cycle=1)
    # Defintion: will rotate the robot to a specified global angle. angle does not need to be between 0 and 2 pi
    # function will treat angles outside this range as though they are
    def drive_rotate_to_angle(self, angle, w_desired):
        alpha = angle % (2*np.pi) - self.th % (2*np.pi) # convert both to angles between 0 and 2 pi
        if (abs(alpha) > np.pi):
            alpha = 2*pi - abs(alpha)
        self.drive_rotate_for_angle(alpha, w_desired)
    # drive_to_point(self, dest_x, dest_y)
    # Defintion: will drive the robot to the destination location, in a straight line
    def drive_to_point(self, dest_x, dest_y, v_desired, w_desired):
        delta_x = dest_x - self.x
        delta_y = dest_y - self.y
        angle = np.arctan2(delta_y,delta_x)
        self.drive_rotate_to_angle(angle,w_desired)
        distance = np.sqrt(delta_x**2 + delta_y**2)
        self.drive_forward_for_distance(distance,v_desired)

class Motor:
    def __init__(self, pwm_output, direction_output):
        self.PWM = pwm_output
        self.DIR = direction_output
