import wiringpi as wp
from time import sleep
from bd import BlueDot

# Class Robot.
from QTR5RC import QTR5RC


def clamped(v, minimum, maximum):
    return max(minimum, min(maximum, v))


class Robot:
    def __init__(self, left_motor_pins, right_motor_pins, line_sensors, base_speed, max_speed, Kp, Kd):
        self.left_motor_pins = left_motor_pins
        self.right_motor_pins = right_motor_pins
        self.Kp = Kp
        self.Kd = Kd
        self.base_speed = base_speed
        self.Speed = max_speed
        self.qtr = QTR5RC(line_sensors)
        self.qtr.initialise_calibration()
        self.last_error = 0

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

    def go(self):
        position = self.qtr.read_line()
        print(f"position {position}")
        error = position - 2000
        motor_speed = self.Kp * error + self.Kd * (error - self.last_error)
        self.last_error = error

        right_speed = clamped(self.base_speed - motor_speed, 0, self.Speed)
        left_speed = clamped(self.base_speed + motor_speed, 0, self.Speed)
        print(f"left {left_speed} right {right_speed}")

        # wp.softPwmWrite(self.left_motor_pins[0], left_speed)
        # wp.softPwmWrite(self.left_motor_pins[1], 0)
        # wp.softPwmWrite(self.right_motor_pins[0], right_speed)
        # wp.softPwmWrite(self.left_motor_pins[1], 0)

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


def pos_to_values(x, y):
    l = y if x > 0 else y + x
    r = y if x < 0 else y - x
    return clamped(l, -1, 1), clamped(r, -1, 1)

if __name__ == "__main__":
    # (Speed, Kp, Ki, Kd)
    r = Robot(left_motor_pins=(23, 24), right_motor_pins=(
        26, 1), line_sensors=[7, 3, 2, 0, 4], base_speed=40, max_speed=80, Kp=0.1, Kd=2)

    r.calibrate()
    bd = BlueDot()

    while True:
        if bd.is_pressed:
            x, y = bd.position.x, bd.position.y
            left, right = pos_to_values(x, y)
            r.drive(left, right)
        else:
            r.stop()
        # try:
        #     r.go()
        # except Exception:
        #     r.stop()
