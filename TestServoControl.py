import gpiozero
import time
from gpiozero.pins.mock import MockFactory, MockPWMPin
import numpy as np
from gpiozero import PWMOutputDevice

servo = PWMOutputDevice(pin=21,active_high=True,initial_value=0,frequency=10000) # Insert the pin number of the servo output here

# loop through different servo PWM values
servo.value = 0
for i in range(10):
    time.sleep(1)
    servo.value += 1