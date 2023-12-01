import Entity
from queue import Queue
from functions import callReadNano
import traceback
from threading import Thread, Event

DEBUG = True
RERR = 0
PERR = 1
YERR = 2

class AutoDrive(Thread):
    def __init__(self, motors):
        super().__init__()
        self.motors = motors
        self.desired_role = 0
        self.desired_pitch = 0
        self.desired_yaw = 0
        self.Kp = 1.0
        self._stop_event = Event()


    def updateError(self, trailer):
            rErr = (self.desired_role - self.imu[int(trailer.name)][0])*self.Kp
            if rErr != 0:
                trailer.driver1.MotorRun(int(rErr))

    def run(self):
        self._stop_event.clear()
        print("starting autodrive thread")
        while not self._stop_event.is_set():
            if DEBUG:
                from random import random
                self.imu = [[random(),random(),random()],
                            [random(),random(),random()],
                            [random(),random(),random()],
                            [random(),random(),random()],
                            [random(),random(),random()]]
            else:
                self.imu = callReadNano()

            for trailer in Entity.ITrailer.trailer_instances:
                if trailer.name == '1':
                    self.updateError(trailer)
        print("stopped the autodrive thread")

    def stop(self):
        self._stop_event.set()