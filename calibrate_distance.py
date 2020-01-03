from gpiozero import DistanceSensor
from time import sleep

sensor = DistanceSensor(echo=9, trigger=10)
while True:
    print('Distance: ', sensor.distance * 100)
    sleep(1)
