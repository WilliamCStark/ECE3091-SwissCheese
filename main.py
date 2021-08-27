from Robot import Robot
from Robot import Motor
import gpiozero
import time
from multiprocessing import Process, Manager, Queue
from multiprocessing.managers import BaseManager
from gpiozero.pins.mock import MockFactory, MockPWMPin
import pickle
import numpy as np
import queue

# For testing on a PC that isn't the PI. Make sure to comment out when running on pi
gpiozero.Device.pin_factory = MockFactory(pin_class=MockPWMPin)

#sensor = gpiozero.DistanceSensor(echo=1,trigger=7)

something = 2
motor_l = Motor(gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=5)) # using GPIO 12 for PWM, GPIO 5 for direction
motor_r = Motor(gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=6))# using GPIO 13 for PWM, GPIO 6 for direction
encoder_l = gpiozero.RotaryEncoder(a=22, b=27,max_steps=100000)  # using GPIO 22 and GPIO 27 for a and b pins from rotary encoder
encoder_r = gpiozero.RotaryEncoder(a=23, b=24,max_steps=100000)  # using GPIO 23 and GPIO 24 for a and b pins from rotary encoder
robot = Robot(3, 10, motor_l, motor_r, encoder_l, encoder_r)

def DriveToGoal(x, y,q):
    v_desired = 5 # move at 5cm per second
    w_desired = np.pi/3 # rotate a half turn in 3 seconds
    count = 0
    robot.q=q
    robot.drive_to_point(x, y, v_desired, w_desired)
    print("end")
    #q.put("DriveToGoal Finished")

def Test():
    for i in range(3):
        time.sleep(1)
        print("test thread")

if __name__ == '__main__':
    q=Queue()
    x=5 # at 30cm away from origin in x-direction
    y=5# at 30cm away from origin in y-direction
    p = Process(target=DriveToGoal, args=(x,y,q))
    test_other = Process(target=Test)
    # put all processes into this array so the thread manager (main thread) can keep track of them
    processes = [p, test_other]
    print("hi")
    p.start()
    test_other.start()
    contents = q.get()
    while True:
        try:
            contents = q.get(False)
            # If we get here we have recieved something from the queue. Make sure we make the appropriate change
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
    p.join()
    print("done with driving")
    test_other.join()
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
