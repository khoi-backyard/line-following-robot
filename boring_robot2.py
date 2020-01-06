import cv2
import numpy as np
import wiringpi as wp

# Class Robot.
from HCSR04 import HCSR04
from QTR5RC import QTR5RC
from bd import BlueDot

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

        print(f"kp {Kp} kd {Kd} base {base_speed} max {max_speed} distance {avoidance_distance}")

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
        for i in range(0, 100):
            self.qtr.calibrate_sensors()
            wp.delay(20)
        print("calibrating done")

    def follow_the_line(self):
        if self.sonic_sensor.distance() <= self.avoidance_distance:
            self.stop()
            return

        position = self.qtr.read_line()

        if sum(self.qtr.sensorValues) == 5000:
            self.stop()
            return

        print(f"position {position}")
        error = position - 2000
        motor_speed = self.Kp * error + self.Kd * (error - self.last_error)
        self.last_error = error

        right_speed = clamped(self.base_speed - motor_speed, 0, self.Speed)
        left_speed = clamped(self.base_speed + motor_speed, 0, self.Speed)
        print(f"left {left_speed} right {right_speed}")

        wp.softPwmWrite(self.left_motor_pins[0], int(left_speed))
        wp.softPwmWrite(self.left_motor_pins[1], 0)
        wp.softPwmWrite(self.right_motor_pins[0], int(right_speed))
        wp.softPwmWrite(self.left_motor_pins[1], 0)

    def stop(self):
        wp.softPwmWrite(self.right_motor_pins[1], 0)
        wp.softPwmWrite(self.left_motor_pins[1], 0)
        wp.softPwmWrite(self.right_motor_pins[0], 0)
        wp.softPwmWrite(self.left_motor_pins[0], 0)

    def drive(self, left, right):
        """left, right take value from -1 1"""
        wp.softPwmWrite(self.left_motor_pins[0], int(left * 100) if left > 0 else 0)
        wp.softPwmWrite(self.left_motor_pins[1], -int(left * 100) if left < 0 else 0)
        wp.softPwmWrite(self.right_motor_pins[0], int(right * 100) if right > 0 else 0)
        wp.softPwmWrite(self.right_motor_pins[1], -int(right * 100) if right < 0 else 0)

    def detect_color(self):
        cap = cv2.VideoCapture(0)

        while 1:
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

            for color_key, contours in contours_dict.items():
                print(type(contours))
                for pic, contour in enumerate(contours):
                    area = cv2.contourArea(contour)
                    if area > 300:
                        print(f"{color_key} {len(contours)}")


if __name__ == "__main__":
    bd = BlueDot()
    r = Robot(left_motor_pins=(23, 24),
              right_motor_pins=(26, 1),
              line_sensors=[7, 3, 2, 0, 4],
              sonic_sensor=HCSR04(echo_pin=13, trigger_pin=12),
              avoidance_distance=15,
              base_speed=20,
              max_speed=40,
              Kp=0.2,
              Kd=10)


    r.stop()
    r.calibrate()
    # r.detect_color()

    while True:
        if bd.is_pressed:
            x, y = bd.position.x, bd.position.y
            left, right = pos_to_values(x, y)
            r.drive(left, right)
        else:
            try:
                r.follow_the_line()
            except KeyboardInterrupt:
                r.stop()
