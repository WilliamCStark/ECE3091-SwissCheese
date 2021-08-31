import gpiozero
import time

# You can replace the pin with whichever pin you are using to drive the PWM input to the motor driver
encoder = gpiozero.RotaryEncoder(a=5, b=6,max_steps=100000)

pre_steps = 0
while True:
    time.sleep(5.0)
    print('Counter:',encoder.steps,'Speed:',(encoder.steps-pre_steps)/5.0,'steps per second\n')
    pre_steps = encoder.steps
