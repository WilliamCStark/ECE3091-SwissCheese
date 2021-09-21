import gpiozero
import time
from gpiozero.pins.mock import MockFactory, MockPWMPin
import numpy as np
from gpiozero import Servo

servo = Servo(17) # Insert the pin number of the servo output here

while True:
    servo.min()
    time.sleep(1)
    servo.mid()
    time.sleep(1)
    servo.max()
    time.sleep(1)