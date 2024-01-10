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
from Events import KeyboardEvents
from combinedMotions import CombinedMotions
from gpiozero import CPUTemperature
import math


KEEP_ALIVE_TIMEOUT_SEC = 1.0

RC = "REMOTE"  # RC=Remote Control
stopVideo = False

startTS = time.time()

nanoTelemetry = {"imu":[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]],"batteryRead":0, "Tcell":0, "m1CS3":0, "m2CS3":0}

# Events
KEEP_ALIVE = 300
LEFT_JOYSTICK = 0
RIGHT_STICK_X = 2
RIGHT_STICK_Y = 3
STOP_ALL = 7
RIGHT_STICK_IN = 8
# SHARE_BUTTON = 49
CIRCLE = 21
TRIANGLE = 23
SHARE_PLUS_OPTIONS = 24
DRIVE1 = 27
DRIVE2 = 28
CHOOSE_PUMP = 29
ACT_PUMP = 30
UP_ARROW = 31
DOWN_ARROW = 32
RIGHT_ARROW = 33
LEFT_ARROW = 34
LED_CTRL = 35


class RobotMain():

    isFlip = False
    isToggle = False
    CurrentJoint = 3
    pitchLoweringActive = False

    def __init__(self) -> None:
        global nanoTelemetry
        
        self.motors = MotorDriver()
        self.camsCB = None
        self.flipCB = None
        self.toggleCb = None
        self.commandCb= None
        self.toggleState = 1
        self.record = False
        self.telemetryChannel = None
        self.ledOn = False
        self.angles = [[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]]
        self.offsets = [[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0],[0, 0, 0]]
        self.recordFileName = "Record"+datetime.datetime.now().strftime("%Y-%m-%d %H:%M")+".csv"
        self.fd = None
        self.writer = None
        self.isAutoDrive = False

        self.joints = [[self.motors.trailer2.turn1,self.motors.trailer1.elevation1],
                  [self.motors.trailer2.turn2,self.motors.trailer2.elevation2],
                  [self.motors.trailer4.turn3,self.motors.trailer4.elevation3],
                  [self.motors.trailer4.turn4,self.motors.trailer5.elevation4]]
        
        self.currPump =0
        print(f"Start robot service {RC}")
        self.currentLightLevel = 1
        self.activePump = 1
        self.isPumpingNow = 0
        self.camsCB = None

        self.comm_thread = threading.Thread(target=self.CommRxHandle)
        self.rx_q = queue.Queue()
        self.autoDrive = AutoDrive(self.motors)
        self.autoDrive.setNanotelemetryCallback(nanoTelemetry)

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

        self.coolerSpeed = 0
        self.motors.MotorRun(self.motors.trailer3.cooler, self.coolerSpeed)
        

    def changeCurrentJoint(self, joint:int):
        RobotMain.CurrentJoint = joint

    def openRecordFile(self):
        self.fd = open(self.recordFileName, 'a')
        self.writer = csv.DictWriter(self.fd)

    def setTelemetryChannel(self, channel):
        self.telemetryChannel = channel

    def setFlipCallback(self, flipCb):
        self.flipCb = flipCb

    def setCommandKB (self, commandCb): #michal - cameras
        self.commandCb = commandCb
        
    def setToggleCallback(self, toggleCb):
        self.toggleCb = toggleCb

    def setCamsCallback(self, camsCb):
        self.camsCB = camsCb

    def A2dHandler(self):
        self.a2d.listen()

    def setCamsCallback(self, camsCb):
        self.camsCB = camsCb

    def CommRxHandle(self):
        if RC == "LOCAL":
            self.ps4Conroller.listen()
        else:
            while True:
                self.comm.start()

    def RobotMessageHandler(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        lastEvent = 0
        while True:
            try:
                events = {}
                while self.rx_q.qsize() > 0:
                    event = self.rx_q.get(0.5)
                    lastEvent = time.time()
                    events[event['event']] = event
                
                if len(events) == 0:
                    if lastEvent > 0 and time.time() - lastEvent > 1: 
                        self.motors.StopAllMotors()
                    time.sleep(0.01)

                for key in events:
                    event = events[key]
                    self.MotorHandler(event)

            except Exception as e:
                traceback.print_exc()

    def runPitchLowering(self):

        trailerNum = 1
        while self.pitchLoweringActive:
            if trailerNum == 1:
                if abs(nanoTelemetry["imu1"][1]) > 2:
                    self.motors.MotorRun(self.motors.trailer1.elevation1, -50)
                else:
                    self.motors.stopMotor(self.motors.trailer1.elevation1)
                    trailerNum += 1
            elif trailerNum == 2:
                if abs(nanoTelemetry["imu2"][1]) > 2:
                    self.motors.MotorRun(self.motors.trailer1.elevation1, 50)
                    self.motors.MotorRun(self.motors.trailer2.elevation2, -50)
                else:
                    self.motors.stopMotor(self.motors.trailer2.elevation1)
                    self.motors.stopMotor(self.motors.trailer2.elevation2)
                    trailerNum += 1
            elif trailerNum == 3:
                if abs(nanoTelemetry["imu3"][1]) > 2:
                    self.motors.MotorRun(self.motors.trailer2.elevation2, 50)
                    self.motors.MotorRun(self.motors.trailer4.elevation3, -50)
                else:
                    self.motors.stopMotor(self.motors.trailer2.elevation2)
                    self.motors.stopMotor(self.motors.trailer4.elevation3)
                    trailerNum += 1
            elif trailerNum == 4:
                if abs(nanoTelemetry["imu4"][1]) > 2:
                    self.motors.MotorRun(self.motors.trailer4.elevation3, 50)
                    self.motors.MotorRun(self.motors.trailer4.elevation4, -50)
                else:
                    self.motors.stopMotor(self.motors.trailer4.elevation3)
                    self.motors.stopMotor(self.motors.trailer4.elevation4)
                    trailerNum += 1
            elif trailerNum == 5:
                if abs(nanoTelemetry["imu5"][1]) > 2:
                    self.motors.MotorRun(self.motors.trailer4.elevation4, 50)
                else:
                    self.motors.stopMotor(self.motors.trailer4.elevation4)
                    trailerNum += 1

        self.motors.stopMotor(self.motors.trailer1.elevation1)
        self.motors.stopMotor(self.motors.trailer2.elevation2)
        self.motors.stopMotor(self.motors.trailer4.elevation3)
        self.motors.stopMotor(self.motors.trailer5.elevation4)

    def pitchLowering(self, value):
        if value == 1:
            self.pitchLoweringActive = True
            pitch_thread = threading.Thread(target=self.runPitchLowering)
            pitch_thread.start()
        else:
            self.pitchLoweringActive = False
    
    def runPitchLeveling(self):
        hysteresis = 1

        needsLeveling = True
        counter = 9000

        with open('../../../../entitiesFlipping.json', 'r') as file:
            json_data = json.load(file)
        reverse_dir_list = json_data.get("reverseDir", [])
        imu_flipping = json_data.get("imu", [])

        correction1 = -1 if (self.motors.trailer2.elevation1.name in reverse_dir_list) else 1
        correction2 = -1 if (self.motors.trailer2.elevation2.name in reverse_dir_list) else 1
        correction3 = -1 if (self.motors.trailer4.elevation3.name in reverse_dir_list) else 1
        correction4 = -1 if (self.motors.trailer4.elevation3.name in reverse_dir_list) else 1
        correction5 = -1 if (self.motors.trailer4.elevation4.name in reverse_dir_list) else 1
        correctionDriver1 = -1 if (self.motors.trailer1.driver1.name in reverse_dir_list) else 1 
        correctionDriver2 = -1 if (self.motors.trailer5.driver2.name in reverse_dir_list) else 1
        imu1Correction = -1 if ("imu1" in imu_flipping) else 1
        imu2Correction = -1 if ("imu2" in imu_flipping) else 1
        imu3Correction = -1 if ("imu3" in imu_flipping) else 1
        imu4Correction = -1 if ("imu4" in imu_flipping) else 1
        imu5Correction = -1 if ("imu5" in imu_flipping) else 1
        
        while needsLeveling:
            counter -= 1
            didSomething = 5
            if (abs(nanoTelemetry["imu1"][1] - self.offsets[0][1]) > hysteresis):
                if ((nanoTelemetry["imu1"][1]*imu1Correction > self.offsets[0][1]*imu1Correction)):
                    self.motors.MotorRun(self.motors.trailer2.elevation1, 70 * correction1)
                else:
                    self.motors.MotorRun(self.motors.trailer2.elevation1, -70 * correction1)
                    self.motors.MotorRun(self.motors.trailer1.driver1, 30 * correctionDriver1)
            else:
                self.motors.stopMotor(self.motors.trailer2.elevation1)
                self.motors.stopMotor(self.motors.trailer1.driver1)
                didSomething -= 1

            if (abs(nanoTelemetry["imu2"][1] - self.offsets[1][1]) > hysteresis):
                if (((nanoTelemetry["imu2"][1]*imu2Correction) > self.offsets[1][1]*imu2Correction)):
                    self.motors.MotorRun(self.motors.trailer2.elevation2, 70 * correction2)
                else:
                    self.motors.MotorRun(self.motors.trailer2.elevation2, -70 * correction2)
            else:
                self.motors.stopMotor(self.motors.trailer2.elevation2)
                didSomething -= 1

            if abs(nanoTelemetry["imu3"][1] - self.offsets[2][1]) > hysteresis:
                print("imu3: ", nanoTelemetry["imu3"][1], "offset3: ", self.offsets[2][1])
                if ((nanoTelemetry["imu3"][1]*imu3Correction > self.offsets[2][1]*imu3Correction)):
                    self.motors.MotorRun(self.motors.trailer4.elevation3, 70 * correction3)
                else:
                    self.motors.MotorRun(self.motors.trailer4.elevation3, -70 * correction3)
            else:
                # self.motors.stopMotor(self.motors.trailer4.elevation3)
                didSomething -= 1
                if abs(nanoTelemetry["imu4"][1] - self.offsets[3][1]) > hysteresis:
                    print("imu4: ", nanoTelemetry["imu4"][1], "offset4: ", self.offsets[3][1])
                    if ((nanoTelemetry["imu4"][1]*imu4Correction > self.offsets[3][1]*imu4Correction)):
                        self.motors.MotorRun(self.motors.trailer4.elevation3, -70 * correction4)
                    else:
                        self.motors.MotorRun(self.motors.trailer4.elevation3, 70 * correction4)
                else:
                    self.motors.stopMotor(self.motors.trailer4.elevation3)
                    didSomething -= 1

            if abs(nanoTelemetry["imu5"][1] - self.offsets[4][1]) > hysteresis:
                print("imu5: ", nanoTelemetry["imu5"][1], "offset5: ", self.offsets[4][1])
                if ((nanoTelemetry["imu5"][1]*imu5Correction > self.offsets[4][1]*imu5Correction)):
                    self.motors.MotorRun(self.motors.trailer4.elevation4, -70 * correction5)
                    self.motors.MotorRun(self.motors.trailer5.driver2, -30 * correctionDriver2)
                else:
                    self.motors.MotorRun(self.motors.trailer4.elevation4, 70 * correction5)
            else:
                self.motors.stopMotor(self.motors.trailer4.elevation4)
                self.motors.stopMotor(self.motors.trailer5.driver2)
                didSomething -= 1
            
            
            if(counter <= 10 or didSomething <= 0):
                needsLeveling = False

        self.motors.stopMotor(self.motors.trailer2.elevation1)
        self.motors.stopMotor(self.motors.trailer2.elevation2)
        self.motors.stopMotor(self.motors.trailer4.elevation3)
        self.motors.stopMotor(self.motors.trailer4.elevation4)
        self.motors.stopMotor(self.motors.trailer1.driver1)
        self.motors.stopMotor(self.motors.trailer5.driver2)

    def pitchLeveling(self, value):
        if value == 1:
            self.pitchLevelingActive = True
            pitch_level_thread = threading.Thread(target=self.runPitchLeveling)
            pitch_level_thread.start()
        else:
            self.pitchLevelingActive = False


    def MotorHandler(self, event):
        with open('../../../../entitiesFlipping.json', 'r') as file:
            json_data = json.load(file)
        reverse_dir_list = json_data.get("reverseDir", [])
        # print(reverse_dir_list)
        
        if event['event'] == KEEP_ALIVE:
            return
        value = int(event["value"])
        # print(value)
        if value > 99:
            value = 99
        elif value < -99:
            value = -99
        e = int(event["event"])
        if int(event["event"]) == RIGHT_STICK_Y:
            # print(event)
            # value = -value ??? it was there
            if value == 0:
                self.motors.stopMotor(self.motors.trailer1.driver1)
                self.motors.stopMotor(self.motors.trailer5.driver2)
            else:
                if self.isFlip:
                    value = -value
                
                if self.motors.trailer1.driver1.name in reverse_dir_list: #checks in json file
                    self.motors.MotorRun(self.motors.trailer1.driver1, -value)
                else:
                    self.motors.MotorRun(self.motors.trailer1.driver1, value)

                if self.motors.trailer5.driver2.name in reverse_dir_list: #checks in json file
                    self.motors.MotorRun(self.motors.trailer5.driver2, -value)
                    
                else:
                    self.motors.MotorRun(self.motors.trailer5.driver2, value)

        elif int(event["event"]) == RIGHT_STICK_X:
            if value == 0:
                if not self.isFlip:
                    self.motors.stopMotor(self.motors.trailer2.turn1)
                else:
                    self.motors.stopMotor(self.motors.trailer4.turn4)
            else:
                if not self.isFlip:
                    if self.motors.trailer2.turn1.name in reverse_dir_list: #checks in json file
                        self.motors.MotorRun(self.motors.trailer2.turn1, -value)
                    else:
                        self.motors.MotorRun(self.motors.trailer2.turn1, value)
                else:
                    if self.motors.trailer4.turn4.name in reverse_dir_list: #checks in json file
                        self.motors.MotorRun(self.motors.trailer4.turn4, value)
                    else:
                        self.motors.MotorRun(self.motors.trailer4.turn4, -value)

        elif int(event["event"]) == TRIANGLE:
            self.isFlip = not self.isFlip

            if (self.isFlip):
                RobotMain.CurrentJoint = 0
            else:
                RobotMain.CurrentJoint = 3

            self.flipCb()

        elif int(event["event"]) in (48, 49, 50, 51, 52):  # michal - cameras
            self.toggleState = int(event["event"])
            self.commandCb(int(event["event"]))

        elif int(event["event"]) == LEFT_ARROW:
            self.motors.stopMotor(self.joints[RobotMain.CurrentJoint][0])
            self.motors.stopMotor(self.joints[RobotMain.CurrentJoint][1])
            
            if self.isFlip:
                if RobotMain.CurrentJoint < 3:
                    RobotMain.CurrentJoint = (RobotMain.CurrentJoint+1)
            else:
                if RobotMain.CurrentJoint > 0:
                    RobotMain.CurrentJoint = RobotMain.CurrentJoint-1
            print(f"Joint number {RobotMain.CurrentJoint} is selected")

        elif int(event["event"]) == RIGHT_ARROW:  # right_arrow
            self.motors.stopMotor(self.joints[RobotMain.CurrentJoint][0])
            self.motors.stopMotor(self.joints[RobotMain.CurrentJoint][1])
            
            if self.isFlip:
                if RobotMain.CurrentJoint > 0:
                    RobotMain.CurrentJoint = RobotMain.CurrentJoint-1
            else:
                if RobotMain.CurrentJoint < 3:
                    RobotMain.CurrentJoint = (RobotMain.CurrentJoint+1)
            print(f"Joint number {RobotMain.CurrentJoint} is selected")

        elif int(event["event"]) == LEFT_JOYSTICK:
            print(RobotMain.CurrentJoint)
            if value == 0:
                self.motors.stopMotor(self.joints[RobotMain.CurrentJoint][0])

            else:
                if self.isFlip:
                    value = -value
                if self.joints[RobotMain.CurrentJoint][0].name in reverse_dir_list: #checks in json file
                    self.motors.MotorRun(self.joints[RobotMain.CurrentJoint][0], -value)
                else:
                    self.motors.MotorRun(self.joints[RobotMain.CurrentJoint][0], value)
                # if not self.isFlip: ---------before
                #     if self.CurrentJoint == 1:
                #         value = -value
                # else:
                #     value = -value

                # self.motors.MotorRun(
                #     self.joints[RobotMain.CurrentJoint][0], value)

        # up_arrow - elevation up for current joint
        elif int(event["event"]) in (UP_ARROW, DOWN_ARROW):

            # if RobotMain.CurrentJoint != 3 and RobotMain.CurrentJoint != 0:
            #     value = -value

            if value == 0:
                self.motors.stopMotor(self.joints[RobotMain.CurrentJoint][1])
                
            else:
                if self.joints[RobotMain.CurrentJoint][1].name in reverse_dir_list: #checks in json file
                    self.motors.MotorRun(self.joints[RobotMain.CurrentJoint][1], -value)     
                else:
                    self.motors.MotorRun(
                        self.joints[RobotMain.CurrentJoint][1], value)

        elif int(event["event"]) == CIRCLE:  # circle - switch sides
            if not self.record:
                self.record = True
            else:
                self.record = False

        elif int(event["event"]) == SHARE_PLUS_OPTIONS:
            self.offsets = self.angles.copy()

        elif int(event["event"]) == CHOOSE_PUMP:

            self.currPump += 1
            if self.currPump > 3:
                self.currPump = 0
        
        elif int(event["event"]) == ACT_PUMP:
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
                    self.motors.MotorRun(self.motors.trailer1.pump1, 100)
                elif self.currPump == 1:
                    self.motors.MotorRun(self.motors.trailer1.pump1, -100)
                elif self.currPump == 2:
                    self.motors.MotorRun(self.motors.trailer5.pump2, 100)
                elif self.currPump == 3:
                    self.motors.MotorRun(self.motors.trailer5.pump2, -100)

        elif int(event["event"]) == LED_CTRL or int(event["event"]) == 25:
            self.ledOn = not self.ledOn
            if self.ledOn:
                self.motors.trailer1.addGpio(13,1)
                self.motors.trailer3.addGpio(17,0)
                self.motors.trailer5.addGpio(13,1)
            else:
                self.motors.trailer1.addGpio(13,0)
                self.motors.trailer3.addGpio(17,1)
                self.motors.trailer5.addGpio(13,0)

        elif int(event["event"]) == DRIVE1:
            # print(event)
            # value = -value
            motor = self.motors.trailer1.driver1
            if self.isFlip:
                value = -value
                motor = self.motors.trailer5.driver2

            if value == 0:
                self.motors.stopMotor(motor)
            else:
                if motor.name in reverse_dir_list: #checks in json file
                    self.motors.MotorRun(motor, -value)
                else:
                    self.motors.MotorRun(motor, value)
                    
        elif int(event["event"]) == DRIVE2:
            # print(event)
            # value = -value
            motor = self.motors.trailer5.driver2
            if self.isFlip:
                value = -value
                motor = self.motors.trailer1.driver1

            if value == 0:
                self.motors.stopMotor(motor)
            else:
                if motor.name in reverse_dir_list: #checks in json file
                    self.motors.MotorRun(motor, -value)
                else:
                    self.motors.MotorRun(motor, value)

        elif int(event["event"]) == STOP_ALL:
            self.motors.StopAllMotors()

        elif int(event["event"]) == RIGHT_STICK_IN:
            curr_time = datetime.now()

            raise NotImplementedError(
                "recording function not implemented yet.")

        # elif int(event["event"]) == SHARE_BUTTON:
        #     if self.isAutoDrive:
        #         self.autoDrive.stop()
        #         self.isAutoDrive = False
        #     else:
        #         self.isAutoDrive = True
        #         self.autoDrive.start()

        elif 32 <= int(event["event"]) <= 122:
            if int(event["event"]) == 117 and value == 1: #'u' keyboard - increase
                self.coolerSpeed = self.coolerSpeed + 1
                if self.coolerSpeed > 99:
                    self.coolerSpeed = 99
                self.motors.MotorRun(self.motors.trailer3.cooler, self.coolerSpeed)

            if int(event["event"]) == 108 and value == 1: #'l' keyboard - decrease
                self.coolerSpeed = self.coolerSpeed - 1

                if self.coolerSpeed < -99:
                    self.coolerSpeed = -99

                self.motors.MotorRun(self.motors.trailer3.cooler, self.coolerSpeed)
            with open('combinedMotios.json', 'r') as file:
                jsonMotions = json.load(file)
                item = jsonMotions.get(chr(e))
                if int(event["event"]) == 105:  # pitch_lowering - i keyboard
                    self.pitchLowering(value)
                elif int(event["event"]) == 42:  # pitch_leveling - * keyboard
                    pass #TODO: after IMU is fixed in 5A we will return to pitch leveling
                    # self.pitchLeveling(value)
                elif int(event["event"]) == 106:  # autodrive - j keyboard
                    if self.isAutoDrive:
                        self.autoDrive.stop()
                        self.isAutoDrive = False
                    else:
                        self.isAutoDrive = True
                        self.autoDrive.start()
                else:
                    if value == 1:
                        CombinedMotions.combinedMotionsMotorRun(
                            item.get("motors"), item.get("speed"))
                    else:
                        CombinedMotions.combinedMotionsMotorStop(
                            item.get("motors"))

        elif e == RIGHT_STICK_IN:
            curr_time = datetime.now()
            self.append_to_csv(nanoTelemetry)
            raise NotImplementedError(
                "recording function not implemented yet.")

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
            callReadNano(ITrailer.trailer_instances, nanoTelemetry, IMotor.motor_instances, False)

    def append_to_csv(self, data):
        with open(self.recordFileName, 'a', newline='') as csvfile:
            self.writer.writerow(data)


    def TelemetricInfoSend(self):
        global nanoTelemetry
        info = {}
        spare_dict = {"Spare"+str(i):4096 for i in range (2,8)}
        front_cameras_dict = {"Camera-F"+str(i):True for i in range(1,5)}
        side_cameras_dict = {"Camera-S"+str(i):True for i in range(1,5)}

        #----calculating total length FO
        bobbin_min_diameter = 50 #D
        bobbin_length = 180 #L
        with open('../../../../entitiesFlipping.json', 'r') as file:
            json_data = json.load(file)
        fiber_diameter_list = json_data.get("fiber_diameter", [])
        if ("0.6" in fiber_diameter_list):
            fiber_diameter = 0.6 #F
        elif ("0.4" in fiber_diameter_list):
            fiber_diameter = 0.4 #F
        else:
            fiber_diameter = 0.2 #F
        spooler_turns = nanoTelemetry["spoolerTurns"] #N
        turns_per_length = math.floor(bobbin_length/fiber_diameter) #TL
        total_layers = math.floor(spooler_turns/turns_per_length)

        total_length = (math.pi(spooler_turns*(bobbin_min_diameter+2*total_layers*fiber_diameter - turns_per_length*fiber_diameter*math.pow(total_layers,2))))/1000

        info.update(nanoTelemetry) # insert to info the information from nanoTelemetry
        info.update({ # insert more info 
                    "opcode": CommandOpcode.telemetric.name,
                    "activePump": self.currPump+1,
                    "pumpingNow": self.isPumpingNow,
                    "isFlip": self.isFlip,
                    "isToggle": self.isToggle,
                    "currentJoint": RobotMain.CurrentJoint,
                    "CPU_tmp": CPUTemperature().temperature,
                    "battery":nanoTelemetry["batteryRead"],
                    "record": self.record,
                    "toggleState": self.toggleState,
                    "cooler_speed": self.coolerSpeed,
                    "CPU_time" : round(startTS*1000),
                    "spooler_current": nanoTelemetry['m1CS3'],
                    "arranger_current": nanoTelemetry['m2CS3'],
                    "Tcell": nanoTelemetry["Tcell"],
                    "fo_length": total_length
                })

        info.update(spare_dict) # insert static information
        info.update(front_cameras_dict) # insert static information
        info.update(side_cameras_dict) # insert static information

        if 'imu1' in nanoTelemetry: self.angles[0] = nanoTelemetry["imu1"]
        if 'imu2' in nanoTelemetry: self.angles[1] = nanoTelemetry["imu2"]
        if 'imu3' in nanoTelemetry: self.angles[2] = nanoTelemetry["imu3"]
        if 'imu4' in nanoTelemetry: self.angles[3] = nanoTelemetry["imu4"]
        if 'imu5' in nanoTelemetry: self.angles[4] = nanoTelemetry["imu5"]
       
        for i in range(0,5): # ovveride imu with normalised imu
            info["imu"][i] = np.subtract(np.array(self.angles[i]) , np.array(self.offsets[i])).tolist()

        # just correct imu pitch for telemetry--------------------------------------
        with open('../../../../entitiesFlipping.json', 'r') as file:
            json_data = json.load(file)
        imu_flipping = json_data.get("imu", [])

        imu1Correction = -1 if ("imu1" in imu_flipping) else 1
        imu2Correction = -1 if ("imu2" in imu_flipping) else 1
        imu3Correction = -1 if ("imu3" in imu_flipping) else 1
        imu4Correction = -1 if ("imu4" in imu_flipping) else 1
        imu5Correction = -1 if ("imu5" in imu_flipping) else 1

        info["imu"][0][1] *= imu1Correction
        info["imu"][1][1] *= imu2Correction
        info["imu"][2][1] *= imu3Correction
        info["imu"][3][1] *= imu4Correction
        info["imu"][4][1] *= imu5Correction
        #-----------------------------------------------------------------------------------
        # insert info about camera ports
        # if self.camsCB is not None:
        #     queues_list = self.camsCB()
        #     for q in queues_list: # each queue for each video handler of the four
        #         item = q.get()
        #         info[item["port"]]=item["cam_name"]

        if self.telemetryChannel is not None:
            self.telemetryChannel.send_message(json.dumps(info))

if __name__ == "__main__":
    RobotMain()