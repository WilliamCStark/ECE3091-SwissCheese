import pigpio
import time
pi = pigpio.pi()
#pi.set_servo_pulsewidth(25,2300)
angle = 2500
pi.set_servo_pulsewidth(25,angle)
time.sleep(2)
while(angle>1200):
    pi.set_servo_pulsewidth(25,angle)
    time.sleep(0.01)
    angle = angle - 1
    print("Angle is: ",angle)
    
