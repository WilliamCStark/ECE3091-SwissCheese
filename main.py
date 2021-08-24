from Robot import Robot
from Robot import Motor
import gpiozero
import time

sensor = gpiozero.DistanceSensor(echo=1,trigger=7)
