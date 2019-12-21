from threading import Event
from time import sleep

from gpiozero import SmoothedInputDevice, DigitalInputDevice


class QuadratureEncoder(object):
    """
    A simple quadrature encoder class
    Note - this class does not determine direction
    """

    def __init__(self, pin_a, pin_b):
        self._value = 0

        self.encoder_a = DigitalInputDevice(pin_a)
        self.encoder_a.when_activated = self._increment
        self.encoder_a.when_deactivated = self._increment

        self.encoder_b = DigitalInputDevice(pin_b)
        self.encoder_b.when_activated = self._increment
        self.encoder_b.when_deactivated = self._increment

    @property
    def a_value(self):
        return self.encoder_a.value

    def reset(self):
        self._value = 0

    def _increment(self):
        self._value += 1

    @property
    def value(self):
        return self._value


class BoringTapeSensor(SmoothedInputDevice):
    def __init__(self, pin=None, queue_len=50, charge_time=0.002, threshold=0.5, partial=False, pin_factory=None):
        super(BoringTapeSensor, self).__init__(pin=pin, pull_up=False, threshold=threshold, queue_len=queue_len,
                                               sample_wait=0.0, partial=partial, pin_factory=pin_factory)
        try:
            self._discharged_time = None
            self._discharged = Event()
            self._charge_time = charge_time
            self.pin.edges = 'falling'
            self.pin.bounce = None
            self.pin.when_changed = self._cap_discharged
            self._queue.start()
        except:
            self.close()
            raise

    def _cap_discharged(self, ticks, state):
        self._discharged_time = ticks
        self._discharged.set()

    def _read(self):
        # Charge the capacitor
        self.pin.function = 'output'
        self.pin.state = 1
        sleep(self._charge_time)

        # Time the discharge
        start = self.pin_factory.ticks()
        self._discharged_time = None
        self._discharged.clear()
        self.pin.function = "input"
        self._discharged.wait(0.002)
        if self._discharged_time is None:
            return 0.0
        else:
            return min(1, 1 - (BoringTapeSensor.ticks_diff(self._discharged_time, start) / self._charge_time))

    @staticmethod
    def ticks_diff(later, earlier):
        return (later - earlier) % 0x100000000

    @property
    def value(self):
        return super(BoringTapeSensor, self).value

    @property
    def line_detected(self):
        return not self.is_active


BoringTapeSensor.when_line = BoringTapeSensor.when_deactivated
BoringTapeSensor.when_no_line = BoringTapeSensor.when_activated
