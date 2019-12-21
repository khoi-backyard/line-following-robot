import line_sensor
from signal import pause
from time import sleep

from signal import pause
from gpiozero import Robot

mid_sensor = line_sensor.BoringTapeSensor(27)
# mid_sensor.when_line = lambda: print('mid Line detected')
# mid_sensor.when_no_line = lambda: print('mid No line detected')

left_sensor = line_sensor.BoringTapeSensor(22)
# left_sensor.when_line = lambda: print('left Line detected')
# left_sensor.when_no_line = lambda: print('left No line detected')
#
#
right_sensor = line_sensor.BoringTapeSensor(17)
# right_sensor.when_line = lambda: print('right Line detected')
# right_sensor.when_no_line = lambda: print('right No line detected')
#
most_left_sensor = line_sensor.BoringTapeSensor(4)
# most_left_sensor.when_line = lambda: print('most left Line detected')
# most_left_sensor.when_no_line = lambda: print('most left No line detected')

most_right_sensor = line_sensor.BoringTapeSensor(23)
# most_right_sensor.when_line = lambda: print('most right Line detected')
# most_right_sensor.when_no_line = lambda: print('most right No line detected')
#


while True:
    print(most_left_sensor.line_detected, left_sensor.line_detected,
          mid_sensor.line_detected, right_sensor.line_detected, most_right_sensor.line_detected)
    print(round(most_left_sensor.value, 2), round(left_sensor.value, 2), round(
        mid_sensor.value, 2), round(right_sensor.value, 2), round(most_right_sensor.value, 2))
    sleep(0.2)

robot = Robot(left=(12, 18), right=(13, 19))

pause()
