import Entity
from queue import Queue
from functions import callReadNano
import traceback
from threading import Thread

DEBUG = True

class AutoDrive(Thread):
    def __init__(self, motors, motorQ:Queue):
        super().__init__()
        self.motors = motors
        self.desired_role = 0
        self.desired_pitch = 0
        self.desired_yaw = 0
        self.Kp = 1.0
        self.motorQ = motorQ
        self.priority_lock = None
        self.nanoQueue = Queue
        self.running = False

    def setNanoQueue(self, nanoQueue):
        self.nanoQueue = nanoQueue

    def start(self):
        while True:
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
                rErr = self.desired_role - self.imu[int(trailer.name)]*self.Kp
                self.motorq.put(rErr)
                pErr = self.desired_pitch - self.imu[int(trailer.name)]*self.Kp
                self.motorq.put(pErr)
                yErr = self.desired_yaw - self.imu[int(trailer.name)]*self.Kp
                self.motorq.put(yErr)
                

    def run(self):
        pass

    def stop(self):
        self.nanoQueue = None
        self.priority_lock = None