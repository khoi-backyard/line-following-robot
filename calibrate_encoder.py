
from gpiozero import Robot, DigitalInputDevice
from libs import QuadratureEncoder
from time import sleep

SAMPLETIME = 1
TARGET = 82
KP = 0.02
KD = 0.01
KI = 0.005

m1_speed = 1
m2_speed = 1

e1_prev_error = 0
e2_prev_error = 0

e1_sum_error = 0
e2_sum_error = 0

r = Robot(left=(12, 18), right=(13, 19))
e1 = QuadratureEncoder(26, 20)
e2 = QuadratureEncoder(6, 5)

r.value = (m1_speed, m2_speed)


while True:
    e1_error = TARGET - e1.value
    e2_error = TARGET - e2.value

    m1_speed += e1_error * KP
    m2_speed += e2_error * KP

    m1_speed = max(min(1, m1_speed), 0)
    m2_speed = max(min(1, m2_speed), 0)

    r.value = (m1_speed, m2_speed)

    print("e1 {} e2 {}".format(e1.value, e2.value))
    print("m1 {} m2 {}".format(m1_speed, m2_speed))

    e1.reset()
    e2.reset()

    sleep(SAMPLETIME)
