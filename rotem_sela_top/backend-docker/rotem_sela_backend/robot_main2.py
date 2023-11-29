import threading
import time
import queue
import datetime
from MotorDriver import MotorDriver
import ps4_controller
import robot_remote_control
from robot_remote_control import CommandOpcode
import RPi.GPIO as GPIO
import asyncio
import json
import numpy as np
import traceback
from functions import callReadNano
from Entity import ITrailer, IMotor
import multiprocessing
import csv

KEEP_ALIVE_TIMEOUT_SEC = 1.0

RC = "REMOTE"  # RC=Remote Control
stopVideo = False

nanoTelemetry = {'imu1':[0,0,0],'imu2':[0,0,0],'imu3':[0,0,0],'imu4':[0,0,0],'imu5':[0,0,0],"batteryRead":0}

# Events
KEEP_ALIVE = 99
LEFT_JOYSTICK = 0
RIGHT_STICK_X = 2
RIGHT_STICK_Y = 3
LEFT_STICK_IN = 7
RIGHT_STICK_IN = 8
CIRCLE = 21
TRIANGLE = 23
SHARE_PLUS_OPTIONS = 24
GIVE_NAME = 30
UP_ARROW = 31
DOWN_ARROW = 32
RIGHT_ARROW = 33
LEFT_ARROW = 34
GIVE_NAME2 = 35



class RobotMain():

    telemetryChannel = None
    flipCb = None
    toggleCb = None
    isFlip = False
    isToggle = False
    joints = None
    currJoint = 3
    ledOn = False

    angle1 = [0, 0, 0]
    angle2 = [0, 0, 0]
    angle3 = [0, 0, 0]
    angle4 = [0, 0, 0]
    angle5 = [0, 0, 0]
    offset1 = [0, 0, 0]
    offset2 = [0, 0, 0]
    offset3 = [0, 0, 0]
    offset4 = [0, 0, 0]
    offset5 = [0, 0, 0]
    

    def __init__(self) -> None:
        self.motors = MotorDriver()
        self.camsCB = None
        self.joints = [[self.motors.trailer1.turn1,self.motors.trailer2.elevation1],
                  [self.motors.trailer3.turn2,self.motors.trailer2.elevation2],
                  [self.motors.trailer3.turn3,self.motors.trailer4.elevation3],
                  [self.motors.trailer5.turn4,self.motors.trailer4.elevation4]]
        self.pumps = [self.motors.trailer1.pump1,self.motors.trailer5.pump2]
        self.currPump =0
        print(f"Start robot service {RC}")
        self.currentLightLevel = 1
        self.activePump = 1
        self.isPumpingNow = 0

        self.comm_thread = threading.Thread(target=self.CommRxHandle)
        self.rx_q = queue.Queue()
        # self.tx_q = queue.Queue()
        if RC == "LOCAL":
            self.ps4Conroller = ps4_controller.RobotPS4(
                self.rx_q, interface="/dev/input/js0", connecting_using_ds4drv=False)
        else:
            print(f"Start remote control service")
            self.comm = robot_remote_control.RobotRemoteControl(self.rx_q)

        # try:
        #     self.a2d = serial_a2d.SerialA2D()
        # except Exception as e:
        #     print("NO a2c Error: ", e)
        # self.a2d_thread = threading.Thread(target=self.A2dHandler)
        self.i2c_lock = threading.Lock()
        self.message_handler_thread = threading.Thread(
            target=self.RobotMessageHandler)
        self.main_thread = threading.Thread(target=self.RobotMain)
        self.readADC_thread = threading.Thread(target=self.ReadADC)
        self.last_keep_alive = datetime.datetime.now()

        self.comm_thread.start()
        self.message_handler_thread.start()
        self.main_thread.start()
        # self.a2d_thread.start()
        self.readADC_thread.start()
        self.motors.StopAllMotors()

    def setTelemetryChannel(self, channel):
        self.telemetryChannel = channel

    def setFlipCallback(self, flipCb):
        self.flipCb = flipCb

    def setToggleCallback(self, toggleCb):
        self.toggleCb = toggleCb

    def setCamsCallback(self, camsCb):
        self.camsCB = camsCb

    def A2dHandler(self):
        self.a2d.listen()

    def CommRxHandle(self):
        if RC == "LOCAL":
            self.ps4Conroller.listen()
        else:
            while True:
                self.comm.start()

    def RobotMessageHandler(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        while True:
            try:
                events = {}
                while self.rx_q.qsize() > 0:

                    event = self.rx_q.get(0.5)
                    events[event['event']] = event
                
                for key in events:
                    event = events[key]
                    self.MotorHandler(event)
            except Exception as e:
                traceback.print_exc()

    def MotorHandler(self, event):
        if event['event'] == KEEP_ALIVE:
            pass
        value = int(event["value"])
        if value > 99:
            value = 99
        elif value < -99:
            value = -99

        if int(event["event"]) == RIGHT_STICK_Y:
            # print(event)
            value = -value
            if self.isFlip:
                value = -value
            if value == 0:
                self.motors.stopMotor(self.motors.trailer1.driver1)
                self.motors.stopMotor(self.motors.trailer5.driver2)
            else:
                self.motors.MotorRun(self.motors.trailer1.driver1, value)
                self.motors.MotorRun(self.motors.trailer5.driver2, -value)

        elif int(event["event"]) == RIGHT_STICK_X:
                if value ==0:
                    if not self.isFlip:
                        self.motors.stopMotor(self.motors.trailer1.turn1)
                    else:
                        self.motors.stopMotor(self.motors.trailer5.turn4)
                else:
                    if not self.isFlip:
                        value = -value
                        self.motors.MotorRun(self.motors.trailer1.turn1, value)
                    else:
                        value = -value
                        self.motors.MotorRun(self.motors.trailer5.turn4, value)

        elif int(event["event"]) == TRIANGLE:
            self.isFlip = not self.isFlip

            if(self.isFlip):
                self.currJoint = 0
            else:
                self.currJoint = 3

            self.flipCb()

        elif int(event["event"]) == LEFT_ARROW:
            self.motors.stopMotor(self.joints[self.currJoint][0])
            self.motors.stopMotor(self.joints[self.currJoint][1])

            if self.isFlip:
                if self.currJoint < 3:
                    self.currJoint=(self.currJoint+1)
            else:
                if self.currJoint > 0:
                    self.currJoint=self.currJoint-1
            print(f"Joing number {self.currJoint} is selected")


        elif int(event["event"]) == RIGHT_ARROW: # right_arrow
            self.motors.stopMotor(self.joints[self.currJoint][0])
            self.motors.stopMotor(self.joints[self.currJoint][1])
            if self.isFlip:   
                if self.currJoint > 0:
                    self.currJoint=self.currJoint-1
            else:
                if self.currJoint < 3:
                    self.currJoint=(self.currJoint+1)
            print(f"Joing number {self.currJoint} is selected")


        elif int(event["event"]) == LEFT_JOYSTICK:
            if value ==0:
                self.motors.stopMotor(self.joints[self.currJoint][0])
            elif not self.isFlip:
                
                self.motors.MotorRun(self.joints[self.currJoint][0], value)
            else:
  
                if self.currJoint == 0:
                    value = -value
                
                self.motors.MotorRun(self.joints[self.currJoint][0], -value)
                
        elif int(event["event"]) in (UP_ARROW, DOWN_ARROW): # up_arrow - elevation up for current joint

            if self.currJoint != 3:
                value = -value

            if value == 0:
                self.motors.stopMotor(self.joints[self.currJoint][1])
            else:
                self.motors.MotorRun(self.joints[self.currJoint][1], value)

        elif int(event["event"]) == CIRCLE: # circle - switch sides
            self.toggleCb()

        elif int(event["event"]) == SHARE_PLUS_OPTIONS:
            self.offset1 = self.angle1
            self.offset2 = self.angle2
            self.offset3 = self.angle3
            self.offset4 = self.angle4
            self.offset5 = self.angle5

        # elif int(event["event"]) == 29:
        #     self.currPump = (self.currPump+1) % 4
            
        elif int(event["event"]) == GIVE_NAME: # stop pump
            if value == 0:
                if self.currPump == 0:
                    self.motors.stopMotor(self.motors.trailer1.pump1)
                elif self.currPump == 1:
                    self.motors.stopMotor(self.motors.trailer1.pump1)
                elif self.currPump == 2:
                    self.motors.stopMotor(self.motors.trailer5.pump2)
                elif self.currPump == 3:
                    self.motors.stopMotor(self.motors.trailer5.pump2)
            else:
                if self.currPump == 0:
                    self.motors.MotorRun(self.motors.trailer1.pump1, 90)
                elif self.currPump == 1:
                    self.motors.MotorRun(self.motors.trailer1.pump1, -90)
                elif self.currPump == 2:
                    self.motors.MotorRun(self.motors.trailer5.pump2, 90)
                elif self.currPump == 3:
                    self.motors.MotorRun(self.motors.trailer5.pump2, -90)

                    
        elif int(event["event"]) == GIVE_NAME2:
            self.ledOn = not self.ledOn
            if self.ledOn:
                self.motors.trailer1.addGpio(13,1)
                self.motors.trailer3.addGpio(17,0)
                self.motors.trailer5.addGpio(13,1)
            else:
                self.motors.trailer1.addGpio(13,0)
                self.motors.trailer3.addGpio(17,1)
                self.motors.trailer5.addGpio(13,0)

        elif int(event["event"]) == LEFT_STICK_IN:
            self.motors.StopAllMotors()

        elif int(event["event"]) == RIGHT_STICK_IN:
            curr_time = datetime.now()

            raise NotImplementedError("recording function not implemented yet.")

        
    def KeepAliveHandler(self):
        self.last_keep_alive = datetime.datetime.now()
        self.motors.disable_motors = False

    def CameraHandler(self, event):
        global stopVideo
        if len(event) < 2:
            print("camera arg missing")
            return
        isToggle = event["isToggle"]
        isFlip = event["isFlip"]

        if isFlip and self.flipCb is not None:
            self.isFlip = not self.isFlip
            self.flipCb()

        lightLevel = event["lightLevel"]
        if isToggle and self.toggleCb is not None:
            self.isToggle = not self.isToggle
            self.toggleCb()

        if (lightLevel):
            self.currentLightLevel += 1
            if (self.currentLightLevel == 4):
                self.currentLightLevel = 1
        self.telemetryChannel.send_message(json.dumps(event))

   
    def RobotMain(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        while True:
            # print(self.a2d.values)
            delta = datetime.datetime.now() - self.last_keep_alive
            # if delta.total_seconds() >= KEEP_ALIVE_TIMEOUT_SEC:
            #     self.motors.StopAllMotors()
            self.TelemetricInfoSend()
            time.sleep(0.1)
           
    
    def ReadADC(self):
        global nanoTelemetry
        while True:
            callReadNano(ITrailer.trailer_instances, nanoTelemetry, IMotor.motor_instances)

    def append_to_csv(self, data, filename):
        with open(filename, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=data.keys())
            writer.writerow(data)


    def TelemetricInfoSend(self):
        global nanoTelemetry

        self.angle1 = nanoTelemetry["imu1"]
        # print(self.angle1, self.offset1)
        self.angle2 = nanoTelemetry["imu2"]
        self.angle3 = nanoTelemetry["imu3"]
        self.angle4 = nanoTelemetry["imu4"]
        self.angle5 = nanoTelemetry["imu5"]
        info = {
            "opcode": CommandOpcode.telemetric.name,
            "activePump": self.currPump+1,
            "pumpingNow": self.isPumpingNow,
            "Spare2": 4096,
            "Spare3": 4096,
            "Spare4": 4096,
            "Spare5": 4096,
            "Spare6": 4096,
            "Spare7": 4096,
            "Camera-F1": True,
            "Camera-S1": True,
            "Camera-F2": True,
            "Camera-S2": True,
            "Camera-F3": True,
            "Camera-S3": True,
            "Camera-F4": True,
            "Camera-S4": True,
            "isFlip": self.isFlip,
            "isToggle": self.isToggle,
            "CurrentJoint": self.currJoint,
            "imu-1" : np.subtract(np.array(nanoTelemetry["imu1"]) , np.array(self.offset1)).tolist(),
            "imu-2" : np.subtract(np.array(nanoTelemetry["imu2"]) , np.array(self.offset2)).tolist(),
            "imu-3" : np.subtract(np.array(nanoTelemetry["imu3"]) , np.array(self.offset3)).tolist(),
            "imu-4" : np.subtract(np.array(nanoTelemetry["imu4"]) , np.array(self.offset4)).tolist(),
            "imu-5" : np.subtract(np.array(nanoTelemetry["imu5"]) , np.array(self.offset5)).tolist(),
            "battery":nanoTelemetry["batteryRead"]
        } 
        queues_list = self.camsCB()
        for q in queues_list: # each queue for each video handler of the four
            item = q.get()
            info[item["port"]]=item["cam_name"]

        if self.telemetryChannel is not None:
            self.telemetryChannel.send_message(json.dumps(info))
        self.append_to_csv(nanoTelemetry, "Record"+datetime.datetime.now())

if __name__ == "__main__":
    RobotMain()