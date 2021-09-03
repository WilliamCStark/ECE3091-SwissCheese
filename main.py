from Robot import Robot
from Robot import Motor
import gpiozero
import time
from multiprocessing import Process, Queue, Pipe
from gpiozero.pins.mock import MockFactory, MockPWMPin
import numpy as np
import queue
from gpiozero import DistanceSensor

# For testing on a PC that isn't the PI. Make sure to comment out when running on pi
gpiozero.Device.pin_factory = MockFactory(pin_class=MockPWMPin)

wheel_radius = 2.26 # chuck in actual wheel_radius
wheel_sep = 5 # chuck in actual wheel separation

# Thread for driving to a goal location
def DriveToGoal(x, y, pipe, rob_loc):
    # Create the robot at the correct location
    motor_l = Motor(gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=5)) # using GPIO 12 for PWM, GPIO 5 for direction
    motor_r = Motor(gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=6))# using GPIO 13 for PWM, GPIO 6 for direction
    encoder_l = gpiozero.RotaryEncoder(a=22, b=27,max_steps=100000)  # using GPIO 22 and GPIO 27 for a and b pins from rotary encoder
    encoder_r = gpiozero.RotaryEncoder(a=23, b=24,max_steps=100000)  # using GPIO 23 and GPIO 24 for a and b pins from rotary encoder
    robot = Robot(wheel_radius, wheel_sep, motor_l, motor_r, encoder_l, encoder_r)
    # set it to the correct location
    robot.x = rob_loc[0]
    robot.y = rob_loc[1]
    robot.th = rob_loc[2]
    v_desired = 12 # chuck in the v_desired
    w_desired = np.pi/3 # chuck in desired rotational velocity
    robot.pipe=pipe
    robot.drive_to_point(x, y, v_desired, w_desired)
    pipe.close()

def CollisionAvoid(pipe, rob_loc):
    # Create the robot at the correct location
    motor_l = Motor(gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=5)) # using GPIO 12 for PWM, GPIO 5 for direction
    motor_r = Motor(gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=6))# using GPIO 13 for PWM, GPIO 6 for direction
    encoder_l = gpiozero.RotaryEncoder(a=22, b=27,max_steps=100000)  # using GPIO 22 and GPIO 27 for a and b pins from rotary encoder
    encoder_r = gpiozero.RotaryEncoder(a=23, b=24,max_steps=100000)  # using GPIO 23 and GPIO 24 for a and b pins from rotary encoder
    robot = Robot(wheel_radius, wheel_sep, motor_l, motor_r, encoder_l, encoder_r)
    # set it to the correct location
    robot.x = rob_loc[0]
    robot.y = rob_loc[1]
    robot.th = rob_loc[2]
    v_desired = 12 # chuck in the v_desired
    w_desired = 5 # chuck in desired rotational velocity
    robot.drive_forward_for_distance(10, -v_desired)
    robot.rest_for_time(0.5)
    robot.drive_rotate_for_angle(np.pi/2, w_desired)
    robot.rest_for_time(0.5)
    robot.drive_forward_for_distance(20, v_desired)
    robot.rest_for_time(0.5)
    pipe.close()

# Thread for checking the ultrasonic sensor and reporting collisions
def CheckUltrasonicSensor(pipe):
    sensor = DistanceSensor(echo=18,trigger=17) # echo and trigger on pins 18 and 17
    dt = 0.01 # check every hundredth of a second for a collision
    time_started = time.time()
    while True:
        if sensor.distance < 0.05:
            # if we are less than 5 centimeteres away, a collision is about to occur, we report to the main threads
            pipe.send("Collision")
        # for testing
        # if time.time()-time_started < 1.01 and time.time()-time_started > 0.99:
        #     pipe.send("Collision")
        time.sleep(dt)

## Main thread here
if __name__ == '__main__':
    # set robot starting location
    x, y, th = 0, 0, 0
    goal_x = 30 # at 30cm away from origin in x-direction
    goal_y = 30 # at 30cm away from origin in y-direction
    # Set up the drive to goal process
    driving_pipe_PARENT, driving_pipe_CHILD = Pipe()
    driving_process = Process(target=DriveToGoal, args=(goal_x,goal_y,driving_pipe_CHILD, (x,y,th)))
    driving_process.start()
    driving_pipe_CHILD.close()
    # Set up the sensor process
    sensor_pipe_PARENT, sensor_pipe_CHILD = Pipe()
    sensor_process = Process(target=CheckUltrasonicSensor, args=(sensor_pipe_CHILD,))
    sensor_process.start()
    sensor_pipe_CHILD.close()
    done = False
    # Run this main process which will drive the robot to the goal, ending either when it reaches the goal, or detects a collision
    last_time_printed = time.time()
    collision_avoiding = False
    while not done:
        if time.time() - last_time_printed > 0.5:
            print("x location is: " + str(x))
            print("y location is: " + str(y))
            print("th location is: " + str(th))
            last_time_printed = time.time()
        if driving_pipe_PARENT.poll():
            # checking if we have any data from driving thread
            try:
                msg = driving_pipe_PARENT.recv()
                # update the robot with the updated robot object properties
                x = msg[0]
                y = msg[1]
                th = msg[2]
            except EOFError:
                # the thread has finished it's business, we should finish up now
                if collision_avoiding:
                    # start drive to goal process
                    driving_process.join() 
                    driving_pipe_PARENT, driving_pipe_CHILD = Pipe()
                    driving_process = Process(target=DriveToGoal, args=(goal_x,goal_y,driving_pipe_CHILD, (x,y,th)))
                    driving_process.start()
                    driving_pipe_CHILD.close()
                    collision_avoiding = False
                else:
                    done = True
        if sensor_pipe_PARENT.poll():
            sensor_pipe_PARENT.recv()
            # Sensor thread has detected a collision
            if not collision_avoiding:
                print("COLLIDED")
                driving_pipe_PARENT.send("Die")
                terminated = False
                while not terminated:
                    try:
                        driving_pipe_PARENT.recv()
                    except ConnectionResetError:
                        terminated = True
                    except EOFError:
                        terminated = True
                driving_process.terminate()
                driving_process.join()
                print("Terminated")
                # Start the new driving process (get around the obstacle)
                collision_avoiding = True
                driving_pipe_PARENT, driving_pipe_CHILD = Pipe()
                driving_process = Process(target=CollisionAvoid, args=(driving_pipe_CHILD, (x,y,th)))
                driving_process.start()
                driving_pipe_CHILD.close()
    print("Done with all")
    # end the sensing thread
    sensor_process.terminate()
    sensor_process.join()
