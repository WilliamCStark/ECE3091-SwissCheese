import time
import numpy as np
import gpiozero

class BaseRobot:
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

        self.e_sum_l = 0
        self.e_sum_r = 0

        # handling the registering of tasks
        # registered movement tasks go into the queue. see the class definition for a task
        self.tasks_queue = list() # an example entry (vel_func, w_func, target_duration)
        # all tasks involve some quantity reaching some target, for example,
        # moving forward for 10 seconds. we must store the current elapsed quantity
        self.task_elapsed_quantity = 0

    def motor_drive(self, motor, duty_cycle, dir):
        motor.PWM.value = duty_cycle
        motor.DIR = dir

    def get_encoder_angular_vel(self, encoder, dt):
        # might need to change to accomodate gear ratio
        delta_steps = encoder.steps - self.previous_steps
        self.previous_steps = encoder.StepsToDistance
        return delta_steps/dt

    # Veclocity motion model
    def base_velocity(self):
        v = (self.wl*self.r + self.wr*self.r)/2.0
        w = (self.wl - self.wr)/self.l
        return v, w

    # At the end of a given time step, update the 'pose', essentially update the
    # internally stored position of the robot.
    def pose_update(self, duty_cycle_l, duty_cycle_r, dir_l, dir_r):
        dt = time.time() - self.previous_time
        self.motor_drive(motor_l, duty_cycle_l, dir_l) # drive left motor
        self.motor_drive(motor_l, duty_cycle_l, dir_l) # drive right motor

        self.wl = self.get_encoder_angular_vel(self.encoder_l, dt) # get right motor angular vel
        self.wr = self.get_encoder_angular_vel(self.encoder_r, dt) # get left motor angular vel

        v, w = self.base_velocity() # get the base velocity from wheel rotations

        self.x = self.x + dt*v*np.cos(self.th)
        self.y = self.y + dt*v*np.sin(self.th)
        self.th = self.th + w*dt

        self.previous_time = time.time()

    ################################################################################
    ### Below is the basic drive function. We specify a desired velocity and speed of
    ### rotation.
    ################################################################################

    def drive(self, v_desired, w_desired):
        wl_desired = v_desired/self.wheel_radius + self.wheel_sep*w_desired/2
        wr_desired = v_desired/self.wheel_radius - self.wheel_sep*w_desired/2

        duty_cycle_l,self.e_sum_l = self.p_control(wl_desired,self.wl,self.e_sum_l)
        duty_cycle_r,self.e_sum_r = self.p_control(wr_desired,self.wr,self.e_sum_r)

        # call pose update with the duty cycle. we reparameterise the duty cycle from -1 to 1 into a 0 to 1 with a single
        # flag for direction.
        pose_update(abs(duty_cycle_l), abs(duty_cycle_r), -(abs(duty_cycle_l)/duty_cycle_l-1)/2,-(abs(duty_cycle_r)/duty_cycle_r-1)/2)

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

    # update function used to drive the robot based on the tasks required of it
    # updates the task state and checks for collisions
    def update(self):
        current_task = self.tasks_queue[0]
        if CollisionTakenPlace:
            # For the moment, if a collision happens, cease all operations
            self.tasks_queue = [] # possibility to add complexity to tasks. allow them to produce contingencies for collisions that take place
            self.stop()
        else:
            current_task.UpdateTask(self)
            if current_task.IsTaskCompleted():
                self.tasks_queue.delete(0)
            else:
                # Drive the robot based on the task instructions
                v_desired = current_task.GetVDesired()
                w_desired = curent_task.GetWDesired()
                self.drive(v_desired, w_desired)

class Robot (BaseRobot):
    def __init__(self, wheel_radius, wheel_sep, motor_l, motor_r, rotary_encoder_l, rotary_encoder_r):
        super().__init__(self, wheel_radius, wheel_sep, motor_l, motor_r, rotary_encoder_l, rotary_encoder_r)
    ###############################################################################################
    ### Below are base task registering functions. These attempt to complete some task until completion,
    ### or until the robot 'collides' with something.
    ################################################################################################
    # drive_forward_for_time(self, time, target_duty_cycle=1):
    # Defintion: will drive the robot forward in a straight line for the specified amount of time
    def drive_forward_for_time(self, time, v_desired):
        v_func = lambda s : v_desired
        w_func = lambda s : 0
        target = time
        task = Task(v_func, w_func, target)
        self.tasks_queue.append(task)
    # drive_forward_for_distance(self, distance, target_duty_cycle=1)
    # Defintion: will drive the robot forward in a straight line for the specified distance
    def drive_forward_for_distance(self, distance, v_desired):
        v_func = lambda s : v_desired
        w_func = lambda s : 0
        target = time
        task = Task(v_func, w_func, target, Task.DefaultDistanceUpdater)
        self.tasks_queue.append(task)
    # drive_rotate_for_time(self, time, direction, target_duty_cycle=1)
    # Defintion: will rotate the robot in the specified direction for an amount of time
    def drive_rotate_for_time(self, time, direction, w_desired):
        v_func = lambda s : 0
        w_func = lambda s : w_desired
        target = time
        task = Task(v_func, w_func, target)
        self.tasks_queue.append(task)
    # drive_rotate_for_angle(self, angle, target_duty_cycle=1)
    # Defintion: will rotate the robot by the specified angle
    def drive_rotate_for_angle(self, angle, w_desired):
        v_func = lambda s : 0
        w_func = lambda s : w_desired
        target = angle
        task = Task(v_func, w_func, target, Task.DefaultAngleUpdater)
        self.tasks_queue.append(task)
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
    def drive_to_point(self, dest_x, dest_y, v_desired):
        delta_x = dest_x - self.x
        delta_y = dest_y - self.y
        angle = np.arctan2(delta_y,delta_x)
        self.drive_rotate_to_angle(angle,np.pi/3) # rotate a half turn in 3 seconds
        distance = np.sqrt(delta_x**2 + delta_y**2)
        self.drive_forward_for_distance(distance,30) # travel 30cm per second

class Motor:
    def __init__(self, pwm_output, direction_output):
        self.PWM = pwm_output
        self.DIR = direction_output

# A task consists of a time dependent velocity, and quantity dependent angular Veclocity
# and a target for which the task will stop. The quant_aq_func is a function used
# to get the value to be added to current on each iteration. This effectively allows
# the user to specify a custom quantity. By default, the time elapsed between task
# updates is used, but distance could easily be specified by passing through an
# appropriate function
class Task:
    @staticmethod
    def DefaultTimeUpdater(rob):
        time.time() - rob.previous_time
    @staticmethod
    def DefaultDistanceUpdater(rob):
        return (time.time() - rob.previous_time)*rob.base_velocity()[0]
    @staticmethod
    def DefaultAngleUpdater(rob):
        return (time.time() - rob.previous_time)*rob.base_velocity()[1]
    def __init__(self, v_func, w_func, target, updater_func = Task.DefaultTimeUpdater):
        self.v_func = vel_func
        self.w_func = w_func
        self.target = target
        self.s = 0 # s parameterises the completion of the task. in practice s represents something like time or distance
        self.updater_func = updater_func

    # Update the task by passing through the robot function, task can then use the custom function for calculating the change in the
    # current quantity value
    def UpdateTask(self, rob):
        self.s += quant_aq_func(rob)

    def IsTaskCompleted(self):
        return self.s >= self.target

    def GetVDesired(self):
        return self.vel_func(self.s)

    def GetWDesired(self):
        return self.w_func(self.s)
