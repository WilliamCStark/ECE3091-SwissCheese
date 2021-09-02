from Robot import Robot
from Robot import Motor
import time
import gpiozero
from gpiozero.pins.mock import MockFactory, MockPWMPin

# For testing on a PC that isn't the PI. Make sure to comment out when running on pi
#gpiozero.Device.pin_factory = MockFactory(pin_class=MockPWMPin)

motor_l = Motor(gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=5)) # using GPIO 12 for PWM, GPIO 5 for direction
motor_r = Motor(gpiozero.PWMOutputDevice(pin=13,active_high=True,initial_value=0,frequency=10000), gpiozero.OutputDevice(pin=6))# using GPIO 13 for PWM, GPIO 6 for direction
encoder_l = gpiozero.RotaryEncoder(a=22, b=27,max_steps=100000)  # using GPIO 22 and GPIO 27 for a and b pins from rotary encoder
encoder_r = gpiozero.RotaryEncoder(a=23, b=24,max_steps=100000)  # using GPIO 23 and GPIO 24 for a and b pins from rotary encoder

wheel_radius = 2.26
robot = Robot(wheel_radius, 10, motor_l, motor_r, encoder_l, encoder_r)
# Will drive forward for 10 second
v_desired = 5 # cm per second
robot.drive_forward_for_time(10, v_desired)
