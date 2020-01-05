from QTR5RC import MrBit_QTR_5RC
import wiringpi as wp
from time import sleep


# Class Robot.
def clamped(v, minimum, maximum):
    return max(minimum, min(maximum, v))


class Robot:
    def __init__(self, left_motor_pins, right_motor_pins, line_sensors, Speed, Kp, Kd):
        self.left_motor_pins = left_motor_pins
        self.right_motor_pins = right_motor_pins
        self.Kp = Kp
        self.Kd = Kd
        self.Speed = Speed
        self.qtr = MrBit_QTR_5RC(line_sensors)
        self.qtr.initialise_calibration()

        for i in range(0, 10):
            self.qtr.calibrate_sensors()
            wp.delay(20)

        wp.wiringPiSetup()

        wp.pinMode(self.left_motor_pins[0], 1)
        wp.pinMode(self.left_motor_pins[1], 1)
        wp.pinMode(self.right_motor_pins[0], 1)
        wp.pinMode(self.right_motor_pins[1], 1)

        wp.softPwmCreate(self.left_motor_pins[0], 0, 100)
        wp.softPwmCreate(self.left_motor_pins[1], 0, 100)
        wp.softPwmCreate(self.right_motor_pins[0], 0, 100)
        wp.softPwmCreate(self.right_motor_pins[1], 0, 100)

    def Go(self):
        self.qtr.read_calibrated()
        print(self.qtr.sensorValues)

    def Stop(self):
        wp.softPwmWrite(self.right_motor_pins[1], 0)
        wp.softPwmWrite(self.left_motor_pins[1], 0)
        wp.softPwmWrite(self.right_motor_pins[0], 0)
        wp.softPwmWrite(self.left_motor_pins[0], 0)


if __name__ == "__main__":
    # (Speed, Kp, Ki, Kd)
    RMIT = Robot(left_motor_pins=(23, 24), right_motor_pins=(
        26, 1), line_sensors=[7, 3, 2, 0, 4], Speed=30, Kp=17.8, Kd=3.1)

    while True:
        try:
            RMIT.Go()
        except Exception:
            RMIT.Stop()
