from Robot import Robot, TentaclePlanner, BaseRobot
from Robot import Motor
import gpiozero
import time
from multiprocessing import Process, Queue, Pipe
from gpiozero.pins.mock import MockFactory, MockPWMPin
import numpy as np
import queue
from gpiozero import DistanceSensor

# For testing on a PC that isn't the PI. Make sure to comment out when running on pi
#gpiozero.Device.pin_factory = MockFactory(pin_class=MockPWMPin)

wheel_radius = 2.26 # chuck in actual wheel_radius
wheel_sep = 5 # chuck in actual wheel separation

# Thread for driving to a goal location
def DriveToGoal(x, y, pipe, rob_loc, collisions_pipe):
    # Create the robot at the correct location
    motor_l = Motor(gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=5)) # using GPIO 12 for PWM, GPIO 5 for direction
    motor_r = Motor(gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=6))# using GPIO 13 for PWM, GPIO 6 for direction
    encoder_l = gpiozero.RotaryEncoder(a=22, b=27,max_steps=100000)  # using GPIO 22 and GPIO 27 for a and b pins from rotary encoder
    encoder_r = gpiozero.RotaryEncoder(a=23, b=24,max_steps=100000)  # using GPIO 23 and GPIO 24 for a and b pins from rotary encoder
    dt = 0.01
    robot = BaseRobot(wheel_radius, wheel_sep, motor_l, motor_r, encoder_l, encoder_r, pipe, dt=dt)
    # set it to the correct location
    robot.x = rob_loc[0]
    robot.y = rob_loc[1]
    robot.th = rob_loc[2]
    # Put in code for dealing with driving to the goal with the tentacles
    planner = TentaclePlanner(dt=dt) # uses default max_v and max_w values
    dist_to_goal = np.sqrt((robot.x-x)**2 +  (robot.y-y)**2)
    angle_to_goal = abs(robot.th-th)
    while not (dist_to_goal < 5 and angle_to_goal < 0.05):
        planner.update_collision_data(collisions_pipe) # push new data from the collisions pipe to the planner object
        v, w = planner.plan(x,y,th,robot.x,robot.y,robot.th) # use tentacles to generate a pair of v and w values to drive at
        robot.drive(v,w) # this function sleeps for the sleeps time dt
        dist_to_goal = np.sqrt((robot.x-x)**2 +  (robot.y-y)**2)
        angle_to_goal = abs(robot.th-th)
    # finish up business
    robot.stop()
    robot.pipe.close()
    collisions_pipe.close()

# Thread for checking the ultrasonic sensor and reporting collisions
def CheckUltrasonicSensor(pipe, collisions_pipe):
    front_sensor = DistanceSensor(echo=18,trigger=17) # echo and trigger on pins 18 and 17
    left_sensor = DistanceSensor(echo=16, trigger=26)
    right_sensor = DistanceSensor(echo=14, trigger=15)
    # TODO: add in other distance sensors for reporting
    dt = 0.01 # check every hundredth of a second for a collision
    while True:
        if front_sensor.distance < 0.05:
            # if we are less than 5 centimeteres away, a collision is really about to occur, we report to the main threads
            pipe.send("Collision")
        collisions_pipe.send([front_sensor.distance * 100, left_sensor.distance * 100, right_sensor.distance * 100])
        time.sleep(dt)

## Main thread here
if __name__ == '__main__':
    # set robot starting location
    x, y, th = 0, 0, 0
    goal_x = 30 # at 30cm away from origin in x-direction
    goal_y = 30 # at 30cm away from origin in y-direction
    # Set up the drive to goal process
    driving_pipe_PARENT, driving_pipe_CHILD = Pipe()
    collision_pipe_DRIVE_END, collision_pipe_SENSOR_END = Pipe()
    driving_process = Process(target=DriveToGoal, args=(goal_x,goal_y,driving_pipe_CHILD, (x,y,th),collision_pipe_DRIVE_END))
    driving_process.start()
    driving_pipe_CHILD.close()
    # Set up the sensor process
    sensor_pipe_PARENT, sensor_pipe_CHILD = Pipe()
    sensor_process = Process(target=CheckUltrasonicSensor, args=(sensor_pipe_CHILD,collision_pipe_SENSOR_END))
    sensor_process.start()
    sensor_pipe_CHILD.close()
    done = False
    # Run this main process which will drive the robot to the goal, ending either when it reaches the goal, or detects a collision
    last_time_printed = time.time()
    while not done:
        # Just print the location every half second for debugging purposes
        if time.time() - last_time_printed > 0.5:
            print("x location is: " + str(x))
            print("y location is: " + str(y))
            print("th location is: " + str(th))
            last_time_printed = time.time()
        # main thread handling code
        if driving_pipe_PARENT.poll():
            try:
                # checking if we have any data from driving thread
                msg = driving_pipe_PARENT.recv()
                # update the robot with the updated robot object properties
                x = msg[0]
                y = msg[1]
                th = msg[2]
            except EOFError:
                # if the driving thread ever ends, it's because it reached the goal
                done = True
        if sensor_pipe_PARENT.poll():
            sensor_pipe_PARENT.recv()
            # Sensor thread has detected a collision that was completely unexpected (closer to the obstacle than we should ever get with the tentacles approach), stop everything to avoid losing marks for a collision
            print("COLLIDED")
            driving_pipe_PARENT.send("Die") # this will cause the robot to stop moving
            terminated = False
            while not terminated:
                try:
                    driving_pipe_PARENT.recv()
                except ConnectionResetError:
                    terminated = True
                except EOFError:
                    terminated = True
            done = True
    print("Done with all")
    # end the driving thread
    driving_process.terminate()
    driving_process.join()
    # end the sensing thread
    sensor_process.terminate()
    sensor_process.join()
