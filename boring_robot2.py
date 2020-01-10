import cv2
import numpy as np
import wiringpi as wp
import os

# Class Robot.
from HCSR04 import HCSR04
from QTR5RC import QTR5RC
from bd import BlueDot
from time import sleep
from collections import Counter

low_yellow = np.array([22, 100, 20], np.uint8)
high_yellow = np.array([40, 255, 255], np.uint8)

low_green = np.array([55, 100, 20], np.uint8)
high_green = np.array([80, 255, 255], np.uint8)

low_blue = np.array([100, 100, 20], np.uint8)
high_blue = np.array([140, 255, 255], np.uint8)


def clamped(v, minimum, maximum):
    return max(minimum, min(maximum, v))


def pos_to_values(x, y):
    l = y if x > 0 else y + x
    r = y if x < 0 else y - x
    return clamped(l, -1, 1), clamped(r, -1, 1)


class Robot:
    def __init__(self,
                 left_motor_pins,
                 right_motor_pins,
                 line_sensors,
                 sonic_sensor,
                 avoidance_distance,
                 base_speed,
                 max_speed,
                 Kp,
                 Kd,
                 ):
        self.left_motor_pins = left_motor_pins
        self.right_motor_pins = right_motor_pins
        self.Kp = Kp
        self.Kd = Kd
        self.base_speed = base_speed
        self.Speed = max_speed
        self.qtr = QTR5RC(line_sensors)
        self.qtr.initialise_calibration()
        self.last_error = 0
        self.sonic_sensor = sonic_sensor
        self.avoidance_distance = avoidance_distance

        print(
            f"kp {Kp} kd {Kd} base {base_speed} max {max_speed} distance {avoidance_distance}")

        wp.wiringPiSetup()

        wp.pinMode(self.left_motor_pins[0], 1)
        wp.pinMode(self.left_motor_pins[1], 1)
        wp.pinMode(self.right_motor_pins[0], 1)
        wp.pinMode(self.right_motor_pins[1], 1)

        wp.softPwmCreate(self.left_motor_pins[0], 0, 100)
        wp.softPwmCreate(self.left_motor_pins[1], 0, 100)
        wp.softPwmCreate(self.right_motor_pins[0], 0, 100)
        wp.softPwmCreate(self.right_motor_pins[1], 0, 100)

    def calibrate(self):
        print("Calibrating")
        for _ in range(0, 100):
            self.qtr.calibrate_sensors()
            wp.delay(20)
        print("calibrating done")

    def follow_the_line(self):
        position = self.qtr.read_line()

        error = position - 2000
        motor_speed = self.Kp * error + self.Kd * (error - self.last_error)
        self.last_error = error

        right_speed = int(
            clamped(self.base_speed - motor_speed, 0, self.Speed))
        left_speed = int(clamped(self.base_speed + motor_speed, 0, self.Speed))
        print(
            f"position {position} left {left_speed} right {right_speed} distance {distance}")

        wp.softPwmWrite(self.left_motor_pins[0], left_speed)
        wp.softPwmWrite(
            self.left_motor_pins[1], right_speed // 3 if left_speed == 0 else 0)

        wp.softPwmWrite(self.right_motor_pins[0], right_speed)
        wp.softPwmWrite(
            self.right_motor_pins[1], left_speed // 3 if right_speed == 0 else 0)

    def stop(self):
        wp.softPwmWrite(self.right_motor_pins[1], 0)
        wp.softPwmWrite(self.left_motor_pins[1], 0)
        wp.softPwmWrite(self.right_motor_pins[0], 0)
        wp.softPwmWrite(self.left_motor_pins[0], 0)

    def drive(self, left, right):
        """left, right take value from -1 1"""
        wp.softPwmWrite(self.left_motor_pins[0], int(
            left * 100) if left > 0 else 0)
        wp.softPwmWrite(
            self.left_motor_pins[1], -int(left * 100) if left < 0 else 0)
        wp.softPwmWrite(self.right_motor_pins[0], int(
            right * 100) if right > 0 else 0)
        wp.softPwmWrite(
            self.right_motor_pins[1], -int(right * 100) if right < 0 else 0)

    def rotate_left(self, speed):
        wp.softPwmWrite(self.left_motor_pins[0], 0)
        wp.softPwmWrite(self.left_motor_pins[1], 0)
        wp.softPwmWrite(self.right_motor_pins[0], speed)
        wp.softPwmWrite(self.right_motor_pins[1], 0)

    def rotate_right(self, speed):
        wp.softPwmWrite(self.left_motor_pins[0], speed)
        wp.softPwmWrite(self.left_motor_pins[1], 0)
        wp.softPwmWrite(self.right_motor_pins[0], 0)
        wp.softPwmWrite(self.right_motor_pins[1], 0)

    def detect_color(self):
        cap = cv2.VideoCapture(0)
        for _ in range(0, 50):  # warming up the camera, discard couple first frames
            _, img = cap.read()

        readings = []

        for _ in range(0, 20):  # take 20 readings
            _, img = cap.read()

            # converting frame(img) from BGR (Blue-Green-Red) to HSV (hue-saturation-value)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            yellow_mask = cv2.inRange(hsv, low_yellow, high_yellow)
            yellow = cv2.bitwise_and(img, img, mask=yellow_mask)

            green_mask = cv2.inRange(hsv, low_green, high_green)
            green = cv2.bitwise_and(img, img, mask=green_mask)

            blue_mask = cv2.inRange(hsv, low_blue, high_blue)
            blue = cv2.bitwise_and(img, img, mask=blue_mask)

            # color: (contours, rectangle color)
            contours_dict = {
                "yellow": (cv2.findContours(
                    yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
                )[0]),
                "green": (cv2.findContours(
                    green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
                )[0]),
                "blue": (cv2.findContours(
                    blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
                )[0]),
            }

            biggest_color = None
            biggest_color_area = 0

            for color_key, contours in contours_dict.items():
                for pic, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if area < 10000:
                        continue  # skip small object
                    if area > biggest_color_area:
                        biggest_color_area = area
                        biggest_color = color_key
            readings.append(biggest_color)
        print(f"biggest color {biggest_color} area {biggest_color_area}")
        c = Counter(readings)
        print(f"fread color {c} most common {c.most_common(1)}")
        return c.most_common(1)


state = "manual"  # start out as manual


def switch_to_manual():
    global state
    print("Switching to manual")
    state = "manual"


def switch_to_line():
    global state
    print("Switching to line")
    state = "line"


def switch_to_color():
    global state
    print("Switching to color")
    state = "color"


bd = BlueDot()
bd.when_manual_pressed = switch_to_color
bd.when_auto_pressed = switch_to_line
bd.when_pressed = switch_to_manual

if __name__ == "__main__":
    r = Robot(left_motor_pins=(23, 24),
              right_motor_pins=(26, 1),
              line_sensors=[7, 3, 2, 0, 4],
              sonic_sensor=HCSR04(echo_pin=13, trigger_pin=12),
              avoidance_distance=15,
              base_speed=50,
              max_speed=80,
              Kp=0.2,
              Kd=10)

    r.stop()
    r.calibrate()

    while True:
        if state == "manual" and bd.is_pressed:
            x, y = bd.position.x, bd.position.y
            left, right = pos_to_values(x, y)
            r.drive(left, right)
        elif state == "line":
            distance = r.sonic_sensor.distance()
            print(f"distance {distance}")
            r.follow_the_line()
            if distance <= r.avoidance_distance:
                r.stop()
            elif sum(r.qtr.sensorValues) == 5000:
                print("reached")
                r.stop()
                state = "color"
            else:
                try:
                    r.follow_the_line()
                except KeyboardInterrupt:
                    r.stop()
        elif state == "color":
            print("color detecting")
            color = r.detect_color()
            print(f"color detected {color}")
            if len(color) == 0:
                print("no color detected")
                state = "manual"
            elif color[0][0] == "yellow":
                print("going to yellow")
                while r.sonic_sensor.distance() > 15:
                    r.drive(0.4, 0.4)
                r.stop()
                state = "manual"
            elif color[0][0] == "green":
                print("going to green")
                r.rotate_left(33)
                sleep(1)
                r.stop()
                while r.sonic_sensor.distance() > 15:
                    r.drive(0.4, 0.4)
                state = "manual"
            elif color[0][0] == "blue":
                print("going to blue")
                r.rotate_right(33)
                sleep(1)
                r.stop()
                while r.sonic_sensor.distance() > 15:
                    r.drive(0.4, 0.4)
                r.stop()
                state = "manual"
            else:
                print("no color trying again")

        else:
            r.stop()
