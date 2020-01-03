from signal import pause

from gpiozero import Robot, DistanceSensor
from bd import BlueDot

from libs import BoringTapeSensor

SPEED = 0.15
TURN_SPEED = 0.13
MANUAL_CONTROL_SPEED = 0.25


def pos_to_values(x, y):
    left = y if x > 0 else y + x
    right = y if x < 0 else y - x

    return clamped(left), clamped(right)


def clamped(v):
    return max(-1, min(1, v))


def manual_control():
    while True:
        if bd.is_pressed:
            x, y = bd.position.x, bd.position.y
            yield pos_to_values(x * MANUAL_CONTROL_SPEED, y * MANUAL_CONTROL_SPEED)
        else:
            yield 0, 0


def line_following_engine():
    action = (0, 0)
    prev_state = None
    while True:
        state = (most_left_sensor.line_detected, left_sensor.line_detected,
                 mid_sensor.line_detected, right_sensor.line_detected, most_right_sensor.line_detected)
        distance_in_cm = distance_sensor.distance * 100
        print(f"state {state} distance {distance_in_cm}")
        if distance_in_cm < 15:
            print("about to hit something")
            action = (0, 0)
        elif state == (False, False, True, False, False):
            print("Going straight")
            action = (1 * SPEED, 1 * SPEED)
        elif state == (False, False, True, True, False):
            print("right and mid - turning right")
            action = (1 * TURN_SPEED, 0)
        elif state == (False, False, False, True, False):
            print("right - turning further right")
            action = (1 * TURN_SPEED, 0)
        elif state == (False, True, True, False, False):
            print("left and mid-  turning left")
            action = (0, 1 * TURN_SPEED)
        elif state == (False, True, False, False, False):
            print("left  - turning further left")
            action = (0, 1 * TURN_SPEED)
        elif state == (False, False, True, True, True):
            print("right crazy")
            action = (1.2 * TURN_SPEED, -1 * TURN_SPEED)
        elif state == (True, True, True, False, False):
            print("left crazy")
            action = (-1 * TURN_SPEED, 1.2 * TURN_SPEED)
        else:
            print(f"WTF prev state {prev_state}")

        prev_state = state
        yield action


left_sensor = BoringTapeSensor(22)
mid_sensor = BoringTapeSensor(27)
right_sensor = BoringTapeSensor(17)
most_right_sensor = BoringTapeSensor(23)  # most right
most_left_sensor = BoringTapeSensor(4)  # most left

distance_sensor = DistanceSensor(echo=9, trigger=10)

robot = Robot(left=(13, 19), right=(12, 18))
bd = BlueDot()

manual_control_engine = manual_control()
autonomous_engine = line_following_engine()


def use_manual_control_engine():
    print("Use manual control")
    robot.source = manual_control_engine


def use_autonomous_engine():
    print("Use autonomous control")
    robot.source = autonomous_engine


bd.when_manual_pressed = use_manual_control_engine
bd.when_auto_pressed = use_autonomous_engine

use_autonomous_engine()  # Start out by using the manual control

pause()
