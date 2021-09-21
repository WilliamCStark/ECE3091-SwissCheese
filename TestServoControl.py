import gpiozero
import time
from gpiozero.pins.mock import MockFactory, MockPWMPin
import numpy as np
from gpiozero import Servo

servo = Servo(21) # Insert the pin number of the servo output here

servo.value = -1 # stay at zero
time.sleep(5)
print("move to pickup")
servo.value = -0.5 # rotate 90 degrees to pickup
time.sleep(5)
print("move back")
servo.value = -1