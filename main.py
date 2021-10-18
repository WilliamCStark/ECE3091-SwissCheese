from Robot import Robot, TentaclePlanner, BaseRobot
from Robot import Motor
import gpiozero
import time
from multiprocessing import Process, Queue, Pipe
from gpiozero.pins.mock import MockFactory, MockPWMPin
import numpy as np
import queue
from gpiozero import DistanceSensor
from gpiozero import AngularServo
from gpiozero import LED

# For testing on a PC that isn't the PI. Make sure to comment out when running on pi
#gpiozero.Device.pin_factory = MockFactory(pin_class=MockPWMPin)

wheel_radius = 2.26 # chuck in actual wheel_radius
wheel_sep = 5 # chuck in actual wheel separation
arena_dims = (100, 100) # width and height of arena in cm

# Thread for driving to a goal location
def DriveToGoal(x, y, th, pipe, rob_loc, collisions_pipe):
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
    front_sensor_1 = DistanceSensor(echo=18,trigger=17) # echo and trigger on pins 18 and 17
    front_sensor_2 = DistanceSensor(echo=,trigger=) # change pins to the second front sensor
    left_sensor = DistanceSensor(echo=14, trigger=15)
    right_sensor = DistanceSensor(echo=16, trigger=26)
    # TODO: add in other distance sensors for reporting
    dt = 0.01 # check every hundredth of a second for a collision
    while True:
        if front_sensor_1.distance < 0.05 or front_sensor_2.distance < 0.05:
            # if we are less than 5 centimeteres away, a collision is really about to occur, we report to the main threads
            pipe.send("Collision")
        collisions_pipe.send([front_sensor_1.distance * 100, front_sensor_2.distance * 100, left_sensor.distance * 100, right_sensor.distance * 100])
        time.sleep(dt)

def CameraThread(pipe):
    # Set up all code to interface with camera and feed to model - return the position of the ball bearing
    dt = 0.01 # choose a reasonable target refresh rate
    while True:
        xpos, ypos, width, height = 0.4, 0.4, 0.05, 0.05 # replace with model - normalise according to image size
        bearing_found = False # replace with model
        msg = [bearing_found, xpos, ypos, width, height]
        pipe.send(msg)
        time.sleep(dt)

def DriveToBallBearing(pipe, rob_loc, ball_loc):
    # we must drive in a straight line towards the ball-bearing
    motor_l = Motor(gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=5)) # using GPIO 12 for PWM, GPIO 5 for direction
    motor_r = Motor(gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=6))# using GPIO 13 for PWM, GPIO 6 for direction
    encoder_l = gpiozero.RotaryEncoder(a=22, b=27,max_steps=100000)  # using GPIO 22 and GPIO 27 for a and b pins from rotary encoder
    encoder_r = gpiozero.RotaryEncoder(a=23, b=24,max_steps=100000)  # using GPIO 23 and GPIO 24 for a and b pins from rotary encoder
    dt = 0.01
    robot = BaseRobot(wheel_radius, wheel_sep, motor_l, motor_r, encoder_l, encoder_r, pipe, dt=dt)
    max_v = 12
    max_w = 1
    # set it to the correct location
    robot.x = rob_loc[0]
    robot.y = rob_loc[1]
    robot.th = rob_loc[2]
    # store the latest ball camera position, normalised to the size of the image
    # TODO: factor in losing sight of the ball bearing for a reason other than arriving at it. also maybe we can still see ball-bearing once we've reached it
    bearing_found, xpos, ypos, width, height = ball_loc
    while bearing_found:
        if pipe.poll():
            msg = pipe.recv()
            if msg == "Die":
                robot.stop()
                pipe.close()
                # wait for death
                while True:
                    pass
            bearing_found, xpos, ypos, width, height = msg
        # TODO: add in a way of getting distance of ball_bearing
        # Simple drive towards ball_bearing
        if xpos < 0.4:
            # set bearing left
            v, w = (0.5*max_v, -0.5*max_w)
        elif xpos > 0.6:
            # set bearing right
            v, w = (0.5*max_v, 0.5*max_w)
        else:
            # straight ahead bearing
            v, w = (0.5*max_v, 0)
        robot.drive(v, w) # this function sleeps for dt

def ScanForMarble(pipe, rob_loc, collisions_pipe):
    goals = [(0, 0, 0), (0, -arena_dims[1], np.pi/2), (arena_dims[0], -arena_dims[1], np.pi), (arena_dims[0], 0, 3*np.pi/2)] # the locations of each corner
    for current_corner in range(4):
        # Navigate to the relavent corner with correct heading - NOTE may need to offset from actual corners to avoid collisions, might need to decrease angular range as well
        goal_x, goal_y, goal_th = goals[current_corner]
        driving_pipe_PARENT, driving_pipe_CHILD = Pipe()
        driving_process = Process(target=DriveToGoal, args=(goal_x,goal_y,goal_th,driving_pipe_CHILD, rob_loc,collisions_pipe))
        driving_process.start()
        driving_pipe_CHILD.close()
        # need to check for thread kill requests
        done = False
        while driving_process.is_alive():
            if pipe.poll():
                msg = pipe.recv()
                if msg == "Die":
                    driving_pipe_PARENT.send("Die") # this will cause the robot to stop moving
                    terminated = False
                    while not terminated:
                        try:
                            driving_pipe_PARENT.recv()
                        except ConnectionResetError:
                            terminated = True
                        except EOFError:
                            terminated = True
                    driving_process.terminate()
                    done = True
                    pipe.close()
                else:
                    driving_pipe_PARENT.send(msg)
            elif driving_pipe_PARENT.poll():
                msg = driving_pipe_PARENT.recv() # get the updated robot position from the drive thread
                pipe.send(msg) # send on up to the main thread
                rob_loc = msg # this threads rob_loc needs to be updated
        if done:
            break
        driving_process.join()
        # Once we've got to the corner, rotate through 90 degrees
        new_goal_th = goal_th - np.pi/2
        driving_pipe_PARENT, driving_pipe_CHILD = Pipe()
        driving_process = Process(target=DriveToGoal, args=(goal_x,goal_y,new_goal_th,driving_pipe_CHILD, rob_loc,collisions_pipe))
        driving_process.start()
        driving_pipe_CHILD.close()
        # need to check for thread kill requests
        done = False
        while driving_process.is_alive():
            if pipe.poll():
                msg = pipe.recv()
                if msg == "Die":
                    driving_pipe_PARENT.send("Die") # this will cause the robot to stop moving
                    terminated = False
                    while not terminated:
                        try:
                            driving_pipe_PARENT.recv()
                        except ConnectionResetError:
                            terminated = True
                        except EOFError:
                            terminated = True
                    driving_process.terminate()
                    pipe.close()
                    done = True
                else:
                    driving_pipe_PARENT.send(msg)
            elif driving_pipe_PARENT.poll():
                msg = driving_pipe_PARENT.recv() # get the updated robot position from the drive thread
                pipe.send(msg) # send on up to the main thread
                rob_loc = msg # this threads rob_loc needs to be updated
        if done:
            break
        driving_process.join()
# Thread controlling the marble pickup
def MarblePickup(pipe, rob_loc, collisions_pipe):
    # TODO: add driving code, can add after we've tested basic function
    servo = AngularServo(26, min_pulse_width=0.0005, max_pulse_width=0.0025)
    led = LED(17) # insert the actual pin for the electromagnet control
    # TODO: put code for activating servo to lower arm
    servo.angle = 90 # replace with actual angle
    time.sleep(1)
    # TODO: put code to turn on electromagnet
    led.on()
    time.sleep(1)
    # TODO: put code for activating servo to raise arm
    servo.angle = 0 # replace with actual angle
    time.sleep(1)
    led.off()
    time.sleep(1)

def main_thread():
    # set robot starting location
    x, y, th = 0, 0, 0
    goal_x = 30 # at 30cm away from origin in x-direction
    goal_y = 30 # at 30cm away from origin in y-direction
    goal_th = 0 # align with zero
    # Set up the camera thread
    camera_pipe_PARENT, camera_pipe_CHILD = Pipe()
    camera_process = Process(target=CameraThread, args=(camera_pipe_CHILD,))
    camera_process.start()
    camera_pipe_CHILD.close()
    # Set up the sensor process
    collision_pipe_DRIVE_END, collision_pipe_SENSOR_END = Pipe()
    sensor_pipe_PARENT, sensor_pipe_CHILD = Pipe()
    sensor_process = Process(target=CheckUltrasonicSensor, args=(sensor_pipe_CHILD,collision_pipe_SENSOR_END))
    sensor_process.start()
    sensor_pipe_CHILD.close()
    # Now figure out what to do based on camera information
    msg = camera_pipe_PARENT.recv()
    bearing_found = msg[0] # flag indicating whether the camera has found a bearing
    # flag variables to set state of robot
    scanning = False
    in_pickup = False
    driving_to_marble = False
    if bearing_found:
        # do something if the bearing has been found
        # start drive to bearing thread
        driving_to_marble = True
        driving_pipe_PARENT, driving_pipe_CHILD = Pipe()
        driving_process = Process(target=DriveToBallBearing, args=(driving_pipe_CHILD, (x,y,th), msg))
        driving_process.start()
        driving_pipe_CHILD.close()
    else:
        # do something else if the bearing hasn't been found
        # start scanning thread
        scanning = True
        driving_pipe_PARENT, driving_pipe_CHILD = Pipe()
        driving_process = Process(target=ScanForMarble, args=(driving_pipe_CHILD, (x,y,th),collision_pipe_DRIVE_END))
        driving_process.start()
        driving_pipe_CHILD.close()
    done = False
    # Run this main process which will drive the robot to the goal, ending either when it reaches the goal, or detects a collision
    last_time_printed = time.time()
    while not done:
        # Just print the location every half second for debugging purposes
        if time.time() - last_time_printed > 0.5:
            print("x location is: " + str(x))
            print("y location is: " + str(y))
            print("th location is: " + str(th))
            print("State: ", (scanning, in_pickup, driving_to_marble))
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
                # if the driving thread ends, figure out why and react accordingly
                if scanning:
                    # if we finish scanning, we could not find the marble, we need to end
                    done = True
                else:
                    if in_pickup:
                        # if the pickup thread ends then we've picked up the marble and need to end
                        done = True
                    elif driving_to_marble:
                        # currently, if the driving to marble thread ends, it means it lost sight of the marble, which we've assumed means we've reached it. This might not be entirely accurate
                        # we may need to include a different condition for when we've reached the marble (distance travelled or location of marble in view), or need to consdier the fact
                        # that we can lose sight of the marble for other reasons (obstacle gets in the way) if this happens, we could remember the approximate location of the marble, and try to reposition to that location
                        
                        # start the marble pickup thread
                        in_pickup = True
                        driving_to_marble = False
                        driving_pipe_PARENT, driving_pipe_CHILD = Pipe()
                        driving_process = Process(target=MarblePickup, args=(driving_pipe_CHILD, (x,y,th),collision_pipe_DRIVE_END))
                        driving_process.start()
                        driving_pipe_CHILD.close()
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
            driving_process.terminate()
            done = True
        if camera_pipe_PARENT.poll():
            # deal with camera input
            msg = camera_pipe_PARENT.recv()
            if scanning:
                # check if the new camera data received whilst scanning means we have a marble
                if msg[0]:
                    # we have detected a ball bearing, we should start the drive to bearing thread
                    # we need to kill the scanning thread first
                    driving_pipe_PARENT.send("Die") # this will cause the robot to stop moving
                    driving_process.join()
                    driving_to_marble = True
                    scanning = False
                    driving_pipe_PARENT, driving_pipe_CHILD = Pipe()
                    driving_process = Process(target=DriveToBallBearing, args=(driving_pipe_CHILD, (x,y,th), msg))
                    driving_process.start()
                    driving_pipe_CHILD.close()
            elif driving_to_marble:
                # we are driving to the marble we need to update the position in the thread navigating to the bearing
                driving_pipe_PARENT.send(msg)

    print("Done with all")
    # end the driving thread
    driving_process.join()
    # end the sensing thread
    sensor_process.terminate()
    sensor_process.join()

## Main thread here
if __name__ == '__main__':
    main_thread() # comment out to test individual functions
