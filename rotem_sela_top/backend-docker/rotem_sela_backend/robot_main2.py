import threading
import time
import queue
import datetime
from pwm_rpi import RobotMotor, LIGHT
from MotorDriver import MotorDriver
import ps4_controller
import robot_remote_control
from robot_remote_control import CommandOpcode
import serial_a2d
import RPi.GPIO as GPIO
import asyncio
import json
import numpy as np
from functions import GenericFunctions
from Entity import ITrailer


KEEP_ALIVE_TIMEOUT_SEC = 1.0
A2D_EXISTS = False
IMU1_EXIST = False
IMU2_EXIST = False
IMU3_EXIST = False
IMU4_EXIST = False
IMU5_EXIST = False

RC = "REMOTE"  # RC=Remote Control
stopVideo = False

nanoTelemetry = {}

class RobotMain():

    telemetryChannel = None
    flipCb = None
    toggleCb = None
    isFlip = False
    isToggle = False
    joints = None
    currJoint = 0

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
        joints = [[self.motors.trailer1.turn1,self.motors.trailer2.elevation1],
                  [self.motors.trailer3.turn2,self.motors.trailer2.elevation2],
                  [self.motors.trailer3.turn3,self.motors.trailer4.elevation3],
                  [self.motors.trailer5.turn4,self.motors.trailer4.elevation4]]
        print(f"Start robot service {RC}")
        self.currentLightLevel = 1
        self.activePump = 1
        self.isPumpingNow = 0

        self.comm_thread = threading.Thread(target=self.CommRxHandle)
        self.rx_q = queue.Queue()
        self.tx_q = queue.Queue()
        if RC == "LOCAL":
            self.ps4Conroller = ps4_controller.RobotPS4(
                self.rx_q, interface="/dev/input/js0", connecting_using_ds4drv=False)
        else:
            print(f"Start remote control service")
            self.comm = robot_remote_control.RobotRemoteControl(self.rx_q)

        try:
            self.a2d = serial_a2d.SerialA2D()
        except Exception as e:
            print("NO a2c Error: ", e)
        self.a2d_thread = threading.Thread(target=self.A2dHandler)
        self.i2c_lock = threading.Lock()
        self.message_handler_thread = threading.Thread(
            target=self.RobotMessageHandler)
        self.main_thread = threading.Thread(target=self.RobotMain)
        self.readADC_thread = threading.Thread(target=self.ReadADC)
        self.last_keep_alive = datetime.datetime.now()

        self.comm_thread.start()
        self.message_handler_thread.start()
        self.main_thread.start()
        self.a2d_thread.start()
        self.readADC_thread.start()
        self.motors.StopAllMotors()

    def setTelemetryChannel(self, channel):
        self.telemetryChannel = channel

    def setFlipCallback(self, flipCb):
        self.flipCb = flipCb

    def setToggleCallback(self, toggleCb):
        self.toggleCb = toggleCb

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
                    # if "" in event:
                    # print(event)
                    # if event["opcode"] == CommandOpcode.motor.value:
                    #     self.MotorHandler(event)
                    # if event["opcode"] == CommandOpcode.keep_alive.value:
                    #     self.KeepAliveHandler()
                    # if event["opcode"] == CommandOpcode.camera.value:
                    #     self.CameraHandler(event)
                    # if event["opcode"] == CommandOpcode.pump.value:
                    #     self.PumpHandler(event)
                    # if event["opcode"] == CommandOpcode.acc_calib.value:
                    #     self.CalibrationHandler(event)
                    # if event['opcode'] == CommandOpcode.stop_all.value:
                    #     self.motors.StopAllMotors()
                    
                    self.MotorHandler(event)
            except Exception as e:
                pass

    def MotorHandler(self, event):
        if len(event) < 2:
            # print("motor arg missing")
            return
        value = int(event["value"])
        if value > 99:
            value = 99
        elif value < -99:
            value = -99
        # motor = RobotMotor(event["motor"])
        if int(event["event"]) == 3: # right_stick_y
            print(event)
            
            if self.isFlip:
                value = -value
            if value == 0:
                self.motors.stopMotor(self.motors.trailer1.driver1)
                self.motors.stopMotor(self.motors.trailer5.driver2)
            else:
                self.motors.MotorRun(self.motors.trailer1.driver1, value)
                self.motors.MotorRun(self.motors.trailer5.driver2, value)
        elif int(event["event"]) == 2: #right_stick_x
                print(event)
                if value ==0:
                    if not self.isFlip:
                        self.motors.stopMotor(self.motors.trailer1.turn1)
                    else:
                        self.motors.stopMotor(self.motors.trailer5.turn4)
                else:
                    if not self.isFlip:
                        self.motors.MotorRun(self.motors.trailer1.turn1, value)
                    else:
                        self.motors.MotorRun(self.motors.trailer5.turn4, value)
        elif int(event["event"]) == 23: # right_stick_y
            self.isFlip = not self.isFlip
            self.flipCb()
        elif int(event["event"]) == 34: # left_arrow
            self.motors.stopMotor(self.joints[self.currJoint][0])
            self.currJoint=(self.currJoint+1) % 4
            print(f"Joing number {self.currJoint} is selected")
        elif int(event["event"]) == 33: # right_arrow
            if self.currJoint > 0:
                self.motors.stopMotor(self.joints[self.currJoint][0])
                self.currJoint=self.currJoint-1 % 4
        elif int(event["event"]) == 0: # moving the left joystick
            # print(event)
            if value ==0:
                self.motors.stopMotor(self.joints[self.currJoint][0])
            elif not self.isFlip:
                self.motors.MotorRun(self.joints[self.currJoint][0], value)
            else:
                self.motors.MotorRun(self.joints[self.currJoint][0], -value)
        elif int(event["event"]) in (31, 32): # up_arrow - elevation up for current joint
            if value == 0:
                self.motors.stopMotor(self.joints[self.currJoint][1])
            else:
                self.motors.MotorRun(self.joints[self.currJoint][1], value)
        

        


                
                







        #     if motor == RobotMotor.Turn1:
        #         motor = RobotMotor.Turn2
        #     elif motor == RobotMotor.Turn2:
        #         motor = RobotMotor.Turn1
        #     elif motor == RobotMotor.Drive1 or motor == RobotMotor.Drive2:
        #         speed = -speed

        # if motor == RobotMotor.Turn1:
        #     motor = RobotMotor.Turn2
        # elif motor == RobotMotor.Turn2:
        #     motor = RobotMotor.Turn1
        # elif motor == RobotMotor.Elev1:
        #     motor = RobotMotor.Joint1

        # elif motor == RobotMotor.Joint1:
        #     motor = RobotMotor.Elev1

        # if speed == 0:
        #     self.motors.stopMotor(motor)
        # else:
        #     if motor == RobotMotor.Drive1:
        #         self.motors.MotorRun(motor, speed)
        #         self.motors.MotorRun(RobotMotor.Drive2, speed)
        #     else:
        #         self.motors.MotorRun(motor, speed)

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
            if (self.currentLightLevel < 3):
                GPIO.output(LIGHT, not (GPIO.input(LIGHT)))
            else:
                stopVideo = True
        self.telemetryChannel.send_message(json.dumps(event))

    def PumpHandler(self, event):
        if len(event) < 2:
            print("pump arg missing")
            return
        togglePumps = event["togglePumps"]
        activePumping = event["activePumping"]
        if (togglePumps):
            self.activePump = self.activePump + 1
            if (self.activePump == 4):
                self.activePump = 1
        if (activePumping):
            self.isPumpingNow = 1
            if self.activePump == 1:
                self.motors.MotorRun(RobotMotor.Pump1, 50)
            elif self.activePump == 2:
                self.motors.MotorRun(RobotMotor.Pump2, 50)
            elif self.activePump == 3:
                self.motors.MotorRun(RobotMotor.Pump3, 50)
        else:
            if (self.isPumpingNow):
                self.isPumpingNow = 0
                if self.activePump == 1:
                    self.motors.stopMotor(RobotMotor.Pump1)
                elif self.activePump == 2:
                  self.motors.stopMotor(RobotMotor.Pump2)
                elif self.activePump == 3:
                    self.motors.stopMotor(RobotMotor.Pump3)
        self.telemetryChannel.send_message(json.dumps(event))

    def CalibrationHandler(self, event):
        self.offset1 = self.angle1
        self.offset2 = self.angle2
        self.offset3 = self.angle3
        self.offset4 = self.angle4
        self.offset5 = self.angle5

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
            # read current of motors
           
    
    def ReadADC(self):
        global nanoTelemetry
        while True:
            GenericFunctions.callReadNano(ITrailer.trailer_instances, nanoTelemetry)
            # print(nanoTelemetry)

    def TelemetricInfoSend(self):
        global nanoTelemetry
        info = {
            "opcode": CommandOpcode.telemetric.name,
            # "imu-1": np.subtract(np.array(self.angle1), np.array(self.offset1)).tolist(),
            # "imu-2": np.subtract(np.array(self.angle2), np.array(self.offset2)).tolist(),
            # "imu-3": np.subtract(np.array(self.angle3), np.array(self.offset3)).tolist(),
            # "imu-4": np.subtract(np.array(self.angle4), np.array(self.offset4)).tolist(),
            # "imu-5": np.subtract(np.array(self.angle5), np.array(self.offset5)).tolist(),
            # "drive1": nanoTelemetry['d1_cs']
            # "drive2": self.a2d.values[2],
            "elev": self.a2d.values[4],
            "turn1": self.a2d.values[3],
            "turn2": self.a2d.values[5],
            "joint1": self.a2d.values[6],
            "FullTank1": self.a2d.values[7],
            "FullTank2": self.a2d.values[8],
            "FullTank3": self.a2d.values[9],
            "activePump": self.activePump,
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
            "isToggle": self.isToggle

        } 
        # print(f"Send telemetry ")

        if self.telemetryChannel is not None:
            self.telemetryChannel.send_message(json.dumps(info))
            # print(nanoTelemetry)
            self.telemetryChannel.send_message(json.dumps(nanoTelemetry))

if __name__ == "__main__":
    RobotMain()