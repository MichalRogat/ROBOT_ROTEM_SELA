from functions import GenericFunctions

HIGH = 1
LOW = 1

class MotorDriver():

    def __init__(self, isUglyDriver, pins, checkOverCurrent):
        self.isUglyDriver = isUglyDriver
        self.pins = pins
        self.checkOverCurrent = checkOverCurrent

    def stopMotor(self):
        self.gpio = LOW
        self.pwm = 0
        self.extra = LOW
        GenericFunctions.callDriverFunction(self)

    def motorRun(self, speed):
        self.gpio = HIGH

        if speed >= 90:
            speed = 90
        if speed <= -90:
            speed = -90

        if self.isUglyDriver:
            # Move counterclock
            if speed >= 0:
                self.pwm = speed
                self.extra = HIGH
            elif speed < 0:
            # Move cloclwise
                speed = abs(speed)
                self.pwm = speed
                self.extra = LOW

        elif not self.isUglyDriver:
            # Move clockwise
            if speed >= 0:
                self.pwm = HIGH
                self.extra = speed
            elif speed < 0:
            # Move counterclock
                speed = abs(speed)
                self.pwm = speed
                self.extra = HIGH

        GenericFunctions.callDriverFunction(self)

# Example
# D1 = MotorDriver(True, [12,11,10], 0)
# D1.motorRun(50)
# D1.stopMotor()