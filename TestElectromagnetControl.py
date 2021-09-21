import gpiozero
import time
from gpiozero.pins.mock import MockFactory, MockPWMPin
import numpy as np
from gpiozero import LED

led = LED(17) # insert the pin number of the electromagnet control
while True:
    led.on()
    time.sleep(1)
    led.off()
    time.sleep(1)