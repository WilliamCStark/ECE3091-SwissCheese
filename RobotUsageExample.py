from Robot import Robot
from Robot import Motor

motor_r = Motor(gpiozero.PWMOutputDevice(pin=12,active_high=True,initial_value=0,frequency=50000), direction = gpiozero.OutputDevice(pin=4))
