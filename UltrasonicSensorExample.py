from gpiozero import DistanceSensor

sensor = DistanceSensor(echo=1,trigger=7) # echo and trigger on pins 1 and 7

for j in range(10):
    print('Distance: ', sensor.distance * 100)
    time.sleep(1)
