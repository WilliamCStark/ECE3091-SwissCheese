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

something = 2
motor_l = Motor(gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=5)) # using GPIO 12 for PWM, GPIO 5 for direction
motor_r = Motor(gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=6))# using GPIO 13 for PWM, GPIO 6 for direction
encoder_l = gpiozero.RotaryEncoder(a=22, b=27,max_steps=100000)  # using GPIO 22 and GPIO 27 for a and b pins from rotary encoder
encoder_r = gpiozero.RotaryEncoder(a=23, b=24,max_steps=100000)  # using GPIO 23 and GPIO 24 for a and b pins from rotary encoder
robot = Robot(3, 10, motor_l, motor_r, encoder_l, encoder_r)
#sensor = DistanceSensor(echo=18,trigger=17) # echo and trigger on pins 18 and 17

def DriveToGoal(x, y,q, kill_pipe):
    v_desired = 5 # move at 5cm per second
    w_desired = np.pi/3 # rotate a half turn in 3 seconds
    count = 0
    robot.q=q
    robot.kill_pipe = kill_pipe
    robot.drive_to_point(x, y, v_desired, w_desired)
    print("end")
    #q.put("DriveToGoal Finished")

def Test():
    for i in range(3):
        time.sleep(1)
        print("test thread")

def CheckUltrasonicSensor(q):
    dt = 0.01 # check every hundredth of a second for a collision
    time_started = time.time()
    while True:
        # if sensor.distance < 0.05:
        #     # if we are less than 5 centimeteres away, a collision is about to occur, we report to the main threads
        #     q.put(["Collision"])
        # for testing
        if time.time()-time_started < 1.01 and time.time()-time_started > 0.99:
            q.put(["Collision"])
            break
        time.sleep(dt)


if __name__ == '__main__':
    q=Queue()
    x=5 # at 30cm away from origin in x-direction
    y=5# at 30cm away from origin in y-direction
    dtg_kill_pipe_PARENT, dtg_kill_pipe_CHILD = Pipe()
    drive_to_goal_process = Process(target=DriveToGoal, args=(x,y,q,dtg_kill_pipe_CHILD))
    test_other = Process(target=Test)
    sensor_process = Process(target=CheckUltrasonicSensor, args=(q,))
    # put all processes into this array so the thread manager (main thread) can keep track of them
    processes = [drive_to_goal_process, test_other, sensor_process]
    print("hi")
    for p in processes:
        p.start()
    while True:
        try:
            contents = q.get(False)
            # If we get here we have recieved something from the queue. Make sure we make the appropriate change
            if contents[0] == "RobotDriver":
                # we have recieved the motor driver message, update this threads Robot
                robot.x = contents[1]
                robot.y = contents[2]
                robot.th = contents[3]
                robot.wl = contents[4]
                robot.wr = contents[5]
            elif contents[0] == "Collision":
                # A collision has occurred, for now we just kill the driving thread
                print("COLLIDED")
                dtg_kill_pipe_PARENT.send("Die")
                print(dtg_kill_pipe_PARENT.recv()) # wait to hear back from the thread before killing it
                drive_to_goal_process.terminate()
                print("terminated")
                # Here is where we would start up a new thread to navigate around the obstacle
            elif contents[0] == "GoalReached":
                # The goal has been reached, finish up the Program
                pass
        except queue.Empty:
            pass
        allExited = True
        # if all threads have ended, the thread manager can end
        # making sure the shared queue is empty before terminating the main thread
        for t in processes:
            if t.exitcode is None:
                allExited = False
                break
        if allExited & q.empty():
            break
    print('out')
    for p in processes:
        p.join()
    print("done with driving")
    print(robot.th)
    print("done with all")

# Non-multiprocessing approach
# exit = False
# last_update = time.time()
# dt = 0.01
# while not exit:
#     # robot.start_action
#     if time.time() - last_update > dt:
#         # Update the robot object by a step
#
#         # Check some condition like collision, or being back in line with the goal and act accordingly
#         # (maybe call the object avoidance protocol)
#
#         # abandon if the goal is complete.
