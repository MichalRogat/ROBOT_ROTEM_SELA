import Entity
from queue import Queue
from functions import callReadNano
import traceback
from threading import Thread

DEBUG = True
rErr = 0
pErr = 1
yErr = 2

class AutoDrive(Thread):
    def __init__(self, motors, motorQ:Queue,pRobotCurrentJoint):
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
        self.trailersError = []

    def setCurrentJointCallback(self, CurrentJointCb):
        self.CurrentJointCb = CurrentJointCb

    def setNanoQueue(self, nanoQueue):
        self.nanoQueue = nanoQueue

    def updateError(self, trailer):
            self.trailersError[trailer.name][rErr] = (self.desired_role - self.imu[int(trailer.name)])*self.Kp
            self.trailersError[trailer.name][pErr] = (self.desired_pitch - self.imu[int(trailer.name)])*self.Kp
            self.trailersError[trailer.name][yErr] = (self.desired_yaw - self.imu[int(trailer.name)])*self.Kp

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
                self.updateError(trailer)
                self.sendRoleFixCommand(trailer)

    def run(self):
        pass

    def stop(self):
        self.nanoQueue = None
        self.priority_lock = None

    def sendRoleFixCommand(self, trailer):
        self.nanoQueue.put({
            
        })