from QTR5RC import MrBit_QTR_5RC
import wiringpi as wp
from time import sleep


# Class Robot.
def clamped(v, minimum, maximum):
    return max(minimum, min(maximum, v))


class Robot:
    LINE_SENSOR_THRESHOLD = 999
    Error = 0
    P = 0
    I = 0
    D = 0
    Previous_Error = 0
    PID_value = 0
    left_speed = 0
    right_speed = 0
    # Constructor

    def __init__(self, left_motor_pins, right_motor_pins, line_sensors, Speed, Kp, Ki, Kd):
        self.left_motor_pins = left_motor_pins
        self.right_motor_pins = right_motor_pins
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Speed = Speed
        self.qtr = MrBit_QTR_5RC(line_sensors)

        self.qtr.initialise_calibration()

        for i in range(0, 10):
            self.qtr.calibrate_sensors()
            wp.delay(20)

        self.qtr.emitters_off()
        wp.wiringPiSetup()

        wp.pinMode(self.left_motor_pins[0], 1)
        wp.pinMode(self.left_motor_pins[1], 1)
        wp.pinMode(self.right_motor_pins[0], 1)
        wp.pinMode(self.right_motor_pins[1], 1)

        wp.softPwmCreate(self.left_motor_pins[0], 0, 100)
        wp.softPwmCreate(self.left_motor_pins[1], 0, 100)
        wp.softPwmCreate(self.right_motor_pins[0], 0, 100)
        wp.softPwmCreate(self.right_motor_pins[1], 0, 100)

    def get_line_values(self):
        self.qtr.read_sensors()
        return [1 if v > self.LINE_SENSOR_THRESHOLD else 0 for v in self.qtr.sensorValues]

    def Calculate_Error(self):
        # Turn right
        line_values = self.get_line_values()
        if ((line_values[0] == 0) and (line_values[1] == 0) and (line_values[2] == 0) and (
                line_values[3] == 0) and (line_values[4] == 1)):
            self.Error = 4
        if ((line_values[0] == 0) and (line_values[1] == 0) and (line_values[2] == 0) and (
                line_values[3] == 1) and (line_values[4] == 1)):
            self.Error = 3
        if ((line_values[0] == 0) and (line_values[1] == 0) and (line_values[2] == 0) and (
                line_values[3] == 1) and (line_values[4] == 0)):
            self.Error = 2
        if ((line_values[0] == 0) and (line_values[1] == 0) and (line_values[2] == 1) and (
                line_values[3] == 1) and (line_values[4] == 0)):
            self.Error = 1
        # Turn left
        if ((line_values[0] == 1) and (line_values[1] == 0) and (line_values[2] == 0) and (
                line_values[3] == 0) and (line_values[4] == 0)):
            self.Error = -4
        elif ((line_values[0] == 1) and (line_values[1] == 1) and (line_values[2] == 0) and (
                line_values[3] == 0) and (line_values[4] == 0)):
            self.Error = -3
        elif ((line_values[0] == 0) and (line_values[1] == 1) and (line_values[2] == 0) and (
                line_values[3] == 0) and (line_values[4] == 0)):
            self.Error = -2
        elif ((line_values[0] == 0) and (line_values[1] == 1) and (line_values[2] == 1) and (
                line_values[3] == 0) and (line_values[4] == 0)):
            self.Error = -1
        # Out of line
        elif ((line_values[0] == 0) and (line_values[1] == 0) and (line_values[2] == 0) and (
                line_values[3] == 0) and (line_values[4] == 0)):
            self.Error = -5
        # Center
        elif ((line_values[0] == 0) and (line_values[1] == 0) and (line_values[2] == 1) and (
                line_values[3] == 0) and (line_values[4] == 0)):
            self.Error = 0
        # print(self.Error)

    def Calculate_PID(self):
        self.Calculate_Error()
        self.P = self.Error
        self.I = self.I + self.Error
        self.D = self.Error - self.Previous_Error
        self.Previous_Error = self.Error
        self.PID_value = (self.Kp * self.P) + \
                         (self.Ki * self.I) + (self.Kd * self.D)

    def get_distance(self):
        return 1000

    def Go(self):
        self.Calculate_PID()
        self.left_speed = clamped(self.Speed + self.PID_value, 0, self.Speed)
        self.right_speed = clamped(self.Speed - self.PID_value, 0, self.Speed)
        print("         Right speed", self.right_speed,
              "           Left speed", self.left_speed)

        # wp.softPwmWrite(self.self.left_motor_pins[0], 50)
        # wp.softPwmWrite(self.self.left_motor_pins[1], round(self.right_speed))

        # self.Stop()
        wp.softPwmWrite(self.right_motor_pins[1], 0)
        wp.softPwmWrite(self.left_motor_pins[0], round(self.left_speed))
        wp.softPwmWrite(self.right_motor_pins[0], round(self.right_speed))
        wp.softPwmWrite(self.left_motor_pins[1], 0)

    def Stop(self):
        wp.softPwmWrite(self.right_motor_pins[1], 0)
        wp.softPwmWrite(self.left_motor_pins[1], 0)
        wp.softPwmWrite(self.right_motor_pins[0], 0)
        wp.softPwmWrite(self.left_motor_pins[0], 0)


if __name__ == "__main__":
    # (Speed, Kp, Ki, Kd)
    RMIT = Robot(left_motor_pins=(23, 24), right_motor_pins=(
        26, 1), line_sensors=[7, 3, 2, 0, 4], Speed=30, Kp=17.8, Ki=0, Kd=3.1)

    RMIT.Go()
    sleep(1)
    RMIT.Stop()
    while True:
        try:
            dis = RMIT.get_distance()
            if (dis < 40):
                RMIT.Stop()
            else:
                RMIT.Go()

        except Exception:
            RMIT.Stop()
