import threading
import time
import queue
import datetime
from MotorDriver import MotorDriver
from AutoDrive5 import AutoDrive
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
import csv

KEEP_ALIVE_TIMEOUT_SEC = 1.0

RC = "REMOTE"  # RC=Remote Control
stopVideo = False

nanoTelemetry = {"imu":[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]],"batteryRead":0}

# Events
KEEP_ALIVE = 99
LEFT_JOYSTICK = 0
RIGHT_STICK_X = 2
RIGHT_STICK_Y = 3
LEFT_STICK_IN = 7
RIGHT_STICK_IN = 8
SHARE_BUTTON = 4
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

    isFlip = False
    isToggle = False
    currJoint = 3

    def __init__(self) -> None:
        self.motors = MotorDriver()
        self.camsCB = None
        self.flipCB = None
        self.toggleCb = None
        self.telemetryChannel = None
        CurrentJoint = 3
        self.ledOn = False
        self.angles = [[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]]
        self.offsets = [[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]]
        self.recordFileName = "Record"+datetime.datetime.now().strftime("%Y-%m-%d %H:%M")+".csv"
        self.fd = None
        self.writer = None
        self.isAutoDrive = False

        self.joints = [[self.motors.trailer1.turn1,self.motors.trailer2.elevation1],
                  [self.motors.trailer3.turn2,self.motors.trailer2.elevation2],
                  [self.motors.trailer3.turn3,self.motors.trailer4.elevation3],
                  [self.motors.trailer5.turn4,self.motors.trailer4.elevation4]]
        
        self.currPump =0
        print(f"Start robot service {RC}")
        self.currentLightLevel = 1
        self.activePump = 1
        self.isPumpingNow = 0

        self.comm_thread = threading.Thread(target=self.CommRxHandle)
        self.rx_q = queue.Queue()
        self.autoDrive = AutoDrive(self.motors, self.rx_q)

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
        self.priority_lock = threading.Lock()
        self.isJoystickActive = False
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

    def changeCurrentJoint(self, joint:int):
        RobotMain.CurrentJoint = joint

    def openRecordFile(self):
        self.fd = open(self.recordFileName, 'a')
        self.writer = csv.DictWriter(self.fd)

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

    def MotorHandler(self, event, isAutoDrive):
        if isAutoDrive:
            self.handleAutoDriveCommands(event)
        
        else:
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
                    RobotMain.CurrentJoint = 0
                else:
                    RobotMain.CurrentJoint = 3

                self.flipCb()

            elif int(event["event"]) == LEFT_ARROW:
                self.motors.stopMotor(self.joints[RobotMain.CurrentJoint][0])
                self.motors.stopMotor(self.joints[RobotMain.CurrentJoint][1])

                if self.isFlip:
                    if RobotMain.CurrentJoint < 3:
                        RobotMain.CurrentJoint=(RobotMain.CurrentJoint+1)
                else:
                    if RobotMain.CurrentJoint > 0:
                        RobotMain.CurrentJoint=RobotMain.CurrentJoint-1
                print(f"Joing number {RobotMain.CurrentJoint} is selected")


            elif int(event["event"]) == RIGHT_ARROW: # right_arrow
                self.motors.stopMotor(self.joints[RobotMain.CurrentJoint][0])
                self.motors.stopMotor(self.joints[RobotMain.CurrentJoint][1])
                if self.isFlip:   
                    if RobotMain.CurrentJoint > 0:
                        RobotMain.CurrentJoint=RobotMain.CurrentJoint-1
                else:
                    if RobotMain.CurrentJoint < 3:
                        RobotMain.CurrentJoint=(RobotMain.CurrentJoint+1)
                print(f"Joing number {RobotMain.CurrentJoint} is selected")


            elif int(event["event"]) == LEFT_JOYSTICK:
                if value ==0:
                    self.motors.stopMotor(self.joints[RobotMain.CurrentJoint][0])
                elif not self.isFlip:
                    
                    self.motors.MotorRun(self.joints[RobotMain.CurrentJoint][0], value)
                else:
    
                    if RobotMain.CurrentJoint == 0:
                        value = -value
                    
                    self.motors.MotorRun(self.joints[RobotMain.CurrentJoint][0], -value)
                    
            elif int(event["event"]) in (UP_ARROW, DOWN_ARROW): # up_arrow - elevation up for current joint

                if RobotMain.CurrentJoint != 3:
                    value = -value

                if value == 0:
                    self.motors.stopMotor(self.joints[RobotMain.CurrentJoint][1])
                else:
                    self.motors.MotorRun(self.joints[RobotMain.CurrentJoint][1], value)

            elif int(event["event"]) == CIRCLE: # circle - switch sides
                self.toggleCb()

            elif int(event["event"]) == SHARE_PLUS_OPTIONS:
                self.offsets = self.angles

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
            
            elif int(event["event"]) == SHARE_BUTTON:
                self.autoQueue = queue
                self.autoDrive.setNanoQueue(AutoDrive)
                self.autoDrive.start()

            elif int(event["event"]) == RIGHT_STICK_IN:
                curr_time = datetime.now()
                self.append_to_csv(nanoTelemetry)
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
            callReadNano(ITrailer.trailer_instances, nanoTelemetry, IMotor.motor_instances, True)

    def append_to_csv(self, data):
        with open(self.recordFileName, 'a', newline='') as csvfile:
            self.writer.writerow(data)


    def TelemetricInfoSend(self):
        global nanoTelemetry
        info = {}
        spare_dict = {"Spare"+str(i):4096 for i in range (2,8)}
        front_cameras_dict = {"Camera-F"+str(i):True for i in range(1,5)}
        side_cameras_dict = {"Camera-S"+str(i):True for i in range(1,5)}

        info.update(nanoTelemetry) # insert to info the information from nanoTelemetry
        info.update({ # insert more info 
            "opcode": CommandOpcode.telemetric.name,
            "activePump": self.currPump+1,
            "pumpingNow": self.isPumpingNow,
            "isFlip": self.isFlip,
            "isToggle": self.isToggle,
            "currentJoint": RobotMain.CurrentJoint,
            "battery":nanoTelemetry["batteryRead"],
        })
        info.update(spare_dict) # insert static information
        info.update(front_cameras_dict) # insert static information
        info.update(side_cameras_dict) # insert static information

        self.angles = nanoTelemetry["imu"]
        for i in range(0,4): # ovveride imu with normalised imu
            info["imu"][i] = np.subtract(np.array(self.angles[i]) , np.array(self.offsets[i])).tolist()

        # insert info about camera ports
        if self.camsCB is not None:
            queues_list = self.camsCB()
            for q in queues_list: # each queue for each video handler of the four
                item = q.get()
                info[item["port"]]=item["cam_name"]

        if self.telemetryChannel is not None:
            self.telemetryChannel.send_message(json.dumps(info))
        if self.autoDrive.running:
            self.autoDrive.nanoQueue.put(item=info["imu"])

if __name__ == "__main__":
    RobotMain()