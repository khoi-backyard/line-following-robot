import wiringpi as wp
import time


class HCSR04:
    def __init__(self, echo_pin, trigger_pin):
        self.echo_pin = echo_pin
        self.trigger_pin = trigger_pin
        self.last_read = None
        self.last_read_time = None

        wp.wiringPiSetup()

        wp.pinMode(trigger_pin, 1)
        wp.pinMode(echo_pin, 0)

        wp.digitalWrite(trigger_pin, 0)

    def distance(self):
        if self.last_read_time and time.time() - self.last_read_time < 0.2:
            return self.last_read

        wp.digitalWrite(self.trigger_pin, 1)
        time.sleep(0.00001)
        wp.digitalWrite(self.trigger_pin, 0)

        while wp.digitalRead(self.echo_pin) == 0:
            pulse_start_time = time.time()

        while wp.digitalRead(self.echo_pin) == 1:
            pulse_end_time = time.time()


        pulse_duration = pulse_end_time - pulse_start_time
        distance = round(pulse_duration * 17150, 2)
        self.last_read_time = time.time()
        self.last_read = distance
        return distance


if __name__ == '__main__':
    sonar = HCSR04(echo_pin=13, trigger_pin=12)
    while 1:
        print(sonar.distance())
