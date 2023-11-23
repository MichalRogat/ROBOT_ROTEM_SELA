from functions import GenericFunctions
from Entity import ITrailer, Trailer1
from MotorDriver import MotorDriver
import threading
import time

nanoTelemetry = {}

class RobotMain():
    def __init__(self):
        self.md = MotorDriver()
        threading.Thread(target=self.ReadADC).start()
        # threading.Thread(target=self.testMotor).start()

    def testMotor(self):
        while True:
            self.md.MotorRun(self.md.trailer1.driver1, 90)
            time.sleep(5)
            self.md.stopMotor(self.md.trailer1.driver1)
            time.sleep(5)

    def ReadADC(self):
        global nanoTelemetry
        while True:
            GenericFunctions.callReadNano(ITrailer.trailer_instances, nanoTelemetry)
            

RobotMain()
time.sleep(120)

