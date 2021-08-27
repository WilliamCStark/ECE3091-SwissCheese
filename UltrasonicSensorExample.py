from gpiozero import DistanceSensor
import time

sensor = DistanceSensor(echo=18,trigger=17) # echo and trigger on pins 1 and 7

for j in range(10):
    print('Distance: ', sensor.distance * 100)
    time.sleep(1)
