# Example code taken from the ECE3091 moodle page
import gpiozero
import time

# You can replace the pin with whichever pin you are using to drive the PWM input to the motor driver
# PIN NUMBER IS GPIO PIN NUMBER, NOT PHYSICAL PIN NUMBER ON BOARD
pwm = gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=50000)

for j in range(10):
    pwm.value = j/10
    direction.value = not direction.value
    print('Duty cycle:',pwm.value,'Direction:',direction.value)
    time.sleep(5.0)

pwm.off()
