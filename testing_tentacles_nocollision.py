from Robot import Robot, TentaclePlanner
from Robot import Motor
import gpiozero
import time
from multiprocessing import Process, Queue, Pipe
from gpiozero.pins.mock import MockFactory, MockPWMPin
import numpy as np
import queue
from gpiozero import DistanceSensor


x,y = 30,30
wheel_radius = 2.26 # chuck in actual wheel_radius
wheel_sep = 5 # chuck in actual wheel separation
# Create the robot at the correct location
motor_l = Motor(gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=5)) # using GPIO 12 for PWM, GPIO 5 for direction
motor_r = Motor(gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=6))# using GPIO 13 for PWM, GPIO 6 for direction
encoder_l = gpiozero.RotaryEncoder(a=22, b=27,max_steps=100000)  # using GPIO 22 and GPIO 27 for a and b pins from rotary encoder
encoder_r = gpiozero.RotaryEncoder(a=23, b=24,max_steps=100000)  # using GPIO 23 and GPIO 24 for a and b pins from rotary encoder
dt = 0.01
robot = Robot(wheel_radius, wheel_sep, motor_l, motor_r, encoder_l, encoder_r, dt=dt)
# set it to the correct location
max_v_desired = 12 # chuck in the v_desired
max_w_desired = 5 # chuck in desired rotational velocity
# Put in code for dealing with driving to the goal with the tentacles
planner = TentaclePlanner(dt=dt)
dist_to_goal = np.sqrt((robot.x-x)**2 +  (robot.y-y)**2)
while dist_to_goal > 0.01:
    v, w = planner.plan(x,y,0,robot.x,robot.y,robot.th)
    # rescale from -1 to 1 to -max_v_desired to max_v_desired
    v = max_v_desired*v
    w = max_w_desired*w
    robot.drive(v,w) # this function sleeps for the sleeps time dt
    dist_to_goal = np.sqrt((robot.x-x)**2 +  (robot.y-y)**2)