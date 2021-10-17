import time
import numpy as np
import gpiozero
import queue
import math

class BaseRobot:
    def __init__(self, wheel_radius, wheel_sep, motor_l, motor_r, rotary_encoder_l, rotary_encoder_r, pipe=None, dt=0.01):
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
        self.gear_ratio_l = 32 # need to have a different gear ratio for each wheel
        self.gear_ratio_r = 32

        self.previous_steps_l = rotary_encoder_l.steps
        self.previous_steps_r = rotary_encoder_r.steps
        self.dt = dt # the time to sleep after applying a motor movement

        self.e_sum_l = 0
        self.e_sum_r = 0

        self.pipe = pipe # the pipe to put pertinent robot variables to be delivered to the main thread from an adjacent thread


    def motor_drive(self, motor, duty_cycle, dir):
        #print("in here")
        #print(motor.PWM)
        motor.PWM.value = duty_cycle
        motor.DIR.value = dir
        #print("Duty cycle: " + str(duty_cycle))

    def get_encoder_angular_vel(self, encoder, dt, previous_steps, gear_ratio):
        # might need to change to accomodate gear ratio
        steps_in_rev = 32
        delta_steps = encoder.steps - previous_steps
        previous_steps = encoder.steps # store previous steps
        delta_rots = delta_steps/steps_in_rev * 2*np.pi / gear_ratio # convert to rad/s for the wheel itself
        return (delta_rots/dt, previous_steps)

    # Veclocity motion model
    def base_velocity(self):
        v = (self.wl*self.wheel_radius + self.wr*self.wheel_radius)/2.0
        w = (self.wl - self.wr)/self.wheel_sep
        return v, w

    # At the end of a given time step, update the 'pose', essentially update the
    # internally stored position of the robot.
    def pose_update(self, duty_cycle_l, duty_cycle_r, dir_l, dir_r):
       # duty_cycle_l = 0.5
       # duty_cycle_r = 0.5
       # dir_r = 0
       # dir_l = 0

        dt = self.dt
        self.motor_drive(self.motor_l, duty_cycle_l, dir_l) # drive left motor
        self.motor_drive(self.motor_r, duty_cycle_r, dir_r) # drive right motor
        # print("Left Direction: " + str(dir_l))
        # print("Right Direction: " + str(dir_r))

        self.wl, self.previous_steps_l = self.get_encoder_angular_vel(self.encoder_l, dt, self.previous_steps_l, self.gear_ratio_l) # get right motor angular vel
        self.wr, self.previous_steps_r = self.get_encoder_angular_vel(self.encoder_r, dt, self.previous_steps_r, self.gear_ratio_r) # get left motor angular vel
        self.wl = -self.wl # angular velocity value is backwards, invert to make forwards
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

        #wl_desired = 5
        #wr_desired = 5



        duty_cycle_l,self.e_sum_l = self.p_control(wl_desired,self.wl,self.e_sum_l)
        duty_cycle_r,self.e_sum_r = self.p_control(wr_desired,self.wr,self.e_sum_r)

        # call pose update with the duty cycle. we reparameterise the duty cycle from -1 to 1 into a 0 to 1 with a single
        # flag for direction.
        self.pose_update(abs(duty_cycle_l), abs(duty_cycle_r), int((np.sign(-duty_cycle_l)+1)/2),int((np.sign(-duty_cycle_r)+1)/2)) # last two converts the sign into a direction (0 or 1)
        # Push the change to the robot in this thread to the queue so the main thread may update it's representation
        if self.pipe is not None:
            self.push_to_pipe()
            self.check_death()
        # print("Rotational velocity of left wheel: " + str(self.wl))
        # print("Rotational velocity of right wheel: " + str(self.wr))
        # print("Forward velocity: " + str(self.base_velocity()[0]))
        # print("Rotational velocity: " + str(self.base_velocity()[1]))
        # print("V_desired: " + str(v_desired))
        # print("w_desired: " + str(w_desired))
        # print("x location is: " + str(self.x))
        # print("y location is: " + str(self.y))
        # print("th location is: " + str(self.th))
        time.sleep(self.dt) # sleep after each drive call so we only drive the robot in increments

    # let the robot rest and reset the error values before attempting a new driving action
    def rest(self):
        self.e_sum_l = 0
        self.e_sum_r = 0

        self.pose_update(0, 0, 0, 0) # last two converts the sign into a direction (0 or 1)
        # Push the change to the robot in this thread to the queue so the main thread may update it's representation
        if self.pipe is not None:
            self.push_to_pipe()
            self.check_death()
        time.sleep(self.dt) # sleep after each drive call so we only drive the robot in increments


    # utility function for the drive function, calculates required duty cycle for
    # a desired step velocity and minimizes accumulated error.
    def p_control(self,w_wheel_desired,w_wheel_measured,e_sum):
        kp = 0.1
        ki = 0.01


        duty_cycle = min(max(-0.96,kp*(w_wheel_desired-w_wheel_measured) + ki*e_sum),0.96)

        e_sum = e_sum + (w_wheel_desired-w_wheel_measured)

       # print("Left duty cycle: " + str(duty_cycle))
      #  print("Right duty cycle: " + str(duty_cycle_r))

        return duty_cycle, e_sum

    # immediately arrest the motion of the robot
    def stop(self):
        self.motor_l.PWM.value = 0
        self.motor_r.PWM.value = 0
        print("hello you called stop")

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
    def __init__(self, wheel_radius, wheel_sep, motor_l, motor_r, rotary_encoder_l, rotary_encoder_r, pipe=None, dt=0.01):
        super().__init__(wheel_radius, wheel_sep, motor_l, motor_r, rotary_encoder_l, rotary_encoder_r, pipe)
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
        while abs(current_distance) < distance:
            self.drive(v_desired, 0)
            current_distance += self.base_velocity()[0]*self.dt
            #current_distance += v_desired*self.dt #TEST EDIT
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
        while abs(current_angle) < angle:
            self.drive(0, w_desired)
            current_angle += self.base_velocity()[1]*self.dt
            #current_angle += w_desired*self.dt #TEST EDIT
    # drive_rotate_to_angle(self, angle, target_duty_cycle=1)
    # Defintion: will rotate the robot to a specified global angle. angle does not need to be between 0 and 2 pi
    # function will treat angles outside this range as though they are
    def drive_rotate_to_angle(self, angle, w_desired):
        if (angle < 0):
            angle_limited = angle % (-2*np.pi)
        else:
            angle_limited = angle % (2*np.pi)
        alpha = angle_limited - self.th % (2*np.pi) # convert both to angles between 0 and 2 pi
        print("alpha is: " + str(alpha))
        if (abs(alpha) > np.pi):
            alpha = 2*np.pi - abs(alpha)
        self.drive_rotate_for_angle(abs(alpha), np.sign(alpha)*w_desired)
    # drive_to_point(self, dest_x, dest_y)
    # Defintion: will drive the robot to the destination location, in a straight line
    def drive_to_point(self, dest_x, dest_y, v_desired, w_desired):
        delta_x = dest_x - self.x
        delta_y = dest_y - self.y
        angle = np.arctan2(delta_y,delta_x)
        self.drive_rotate_to_angle(angle,w_desired)
        self.rest_for_time(0.5)
        distance = np.sqrt(delta_x**2 + delta_y**2)
        self.drive_forward_for_distance(distance,v_desired)
        self.rest_for_time(0.5)
        self.drive_rotate_to_angle(0,w_desired)
    def rest_for_time(self, t):
        start_time = time.time()
        while (time.time() - start_time) < t:
            self.rest()

class Motor:
    def __init__(self, pwm_output, direction_output):
        self.PWM = pwm_output
        self.DIR = direction_output

class TentaclePlanner:
    def __init__(self,dt=0.1,steps=5,alpha=1,beta=0.1, max_v=12, max_w=1):
        
        self.dt = dt
        self.steps = steps
        # Tentacles are possible trajectories to follow: NOTE we can try adding higher resolution tentacles to make reaching the goal more smooth
        self.tentacles = [(0.0,max_w),(0.0,-max_w),(max_v,max_w),(max_v,-max_w),(max_v,max_w*0.5),(max_v,-max_w*0.5),(max_v,0.0),(0.0,0.0)]
        
        self.alpha = alpha
        self.beta = beta

        # add in some state variables to track collisions
        self.sensor_front1_distance = 100 # in cm
        self.sensor_front2_distance = 100 # in cm
        self.sensor_left_distance = 100 # in cm
        self.sensor_right_distance = 100 # in cm
        
        #self.obstacles = obstacles
    # Play a trajectory and evaluate where you'd end up
    def roll_out(self,v,w,goal_x,goal_y,goal_th,x,y,th):
        
        for j in range(self.steps):
            x = x + self.dt*v*np.cos(th)
            y = y + self.dt*v*np.sin(th)
            th = (th + w*self.dt)
            sensor_distances = [self.sensor_front1_distance, self.sensor_front2_distance, self.sensor_left_distance, self.sensor_right_distance]
            sensor_labels = ["front1", "front2", "left", "right"]
            # check each sensor for a possible collision
            for i in range(len(sensor_distances)):
                if (sensor_distances[i] < 10):
                    print(sensor_labels[i] + " sensor detected collision! Evaluating tentacle (" + str(v) + ", " + str(w) + ")...")
                    # We have detected an obstacle in this direction, ignore these tentacles
                    if i == 0 or i == 1:
                        # ignore front going tentacles
                        if v > 0:
                            return np.inf
                    if i == 2:
                        # ignore left going tentacles
                        if w < 0:
                            return np.inf
                    if i == 3:
                        # ignore right going tentacles
                        if w > 0:
                            print("Ignoring tentacle: (" + str(v) + ", " + str(w) + ")...")
                            return np.inf
                
        e_th = goal_th-th
        e_th = np.arctan2(np.sin(e_th),np.cos(e_th))
        
        return self.alpha*((goal_x-x)**2 + (goal_y-y)**2) + self.beta*(e_th**2)

    # Choose trajectory that will get you closest to the goal
    def plan(self,goal_x,goal_y,goal_th,x,y,th):
        
        costs =[]
        for v,w in self.tentacles:
            costs.append(self.roll_out(v,w,goal_x,goal_y,goal_th,x,y,th))
        
        best_idx = np.argmin(costs)

        print("Chosen Tentacle: ",  self.tentacles[best_idx])
        
        return self.tentacles[best_idx]

    def update_collision_data(self, collisions_pipe):
        # We must get the latest collision data
        new_collision_data = False
        # Keep polling until no new data is found, update data with the newest data found
        while collisions_pipe.poll():
            new_collision_data = True
            data = collisions_pipe.recv()
        if new_collision_data:
            self.sensor_front1_distance = data[0]
            self.sensor_front2_distance = data[1]
            self.sensor_left_distance = data[2]
            self.sensor_right_distance = data[3]
