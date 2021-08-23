from Robot import Robot
from Robot import Motor
import time

motor_l = Motor(gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=50000), direction = gpiozero.OutputDevice(pin=5)) # using GPIO 12 for PWM, GPIO 5 for direction
motor_r = Motor(gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=50000), direction = gpiozero.OutputDevice(pin=6))# using GPIO 13 for PWM, GPIO 6 for direction
encoder_l = gpiozero.RotaryEncoder(a=22, b=27,max_steps=100000)  # using GPIO 22 and GPIO 27 for a and b pins from rotary encoder
encoder_r = gpiozero.RotaryEncoder(a=23, b=24,max_steps=100000)  # using GPIO 23 and GPIO 24 for a and b pins from rotary encoder

robot = Robot(3, 10, motor_l, motor_r, encoder_l, encoder_r)

# Will drive forward for 1 second
robot.drive_forward_for_time(self, 1000, v_desired)
