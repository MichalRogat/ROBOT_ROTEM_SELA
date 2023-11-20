import threading
import time
import queue
import logging
import os
import datetime
from pwm_rpi import RobotMotor, MotorDriver, LIGHT
import ps4_controller
import robot_remote_control
from robot_remote_control import CommandOpcode
from enum import Enum
from minimu import MinIMU_v5_pi
import serial_a2d
import RPi.GPIO as GPIO
import asyncio
import json
import numpy as np
import math


KEEP_ALIVE_TIMEOUT_SEC = 1.0
A2D_EXISTS = True
IMU1_EXIST = True
IMU2_EXIST = True

RC = "REMOTE"  # RC=Remote Control
stopVideo = False


class RobotMain():

    telemetryChannel = None
    flipCb = None
    toggleCb = None
    isFlip = False
    isToggle = False
    angle1 = [0, 0, 0]
    angle2 = [0, 0, 0]
    offset1 = [0, 0, 0]
    offset2 = [0, 0, 0]

    def __init__(self) -> None:
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

        self.a2d = serial_a2d.SerialA2D()
        self.a2d_thread = threading.Thread(target=self.A2dHandler)
        self.i2c_lock = threading.Lock()
        self.message_handler_thread = threading.Thread(
            target=self.RobotMessageHandler)
        self.main_thread = threading.Thread(target=self.RobotMain)
        self.last_keep_alive = datetime.datetime.now()
        self.motors = MotorDriver()

        if IMU1_EXIST:
            self.imu_1 = MinIMU_v5_pi(0, self.i2c_lock)
            self.imu_1.trackAngle()
            # self.imu_1.trackYaw()

        if IMU2_EXIST:
            self.imu_2 = MinIMU_v5_pi(1, self.i2c_lock, mFullScale=16)
            self.imu_2.trackAngle()
            # self.imu_2.trackYaw()

        self.comm_thread.start()
        self.message_handler_thread.start()
        self.main_thread.start()
        self.a2d_thread.start()
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
                event = self.rx_q.get(0.5)
                if event["opcode"] == CommandOpcode.motor.value:
                    self.MotorHandler(event)
                if event["opcode"] == CommandOpcode.keep_alive.value:
                    self.KeepAliveHandler()
                if event["opcode"] == CommandOpcode.camera.value:
                    self.CameraHandler(event)
                if event["opcode"] == CommandOpcode.pump.value:
                    self.PumpHandler(event)
                if event["opcode"] == CommandOpcode.acc_calib.value:
                    self.CalibrationHandler(event)
                if event['opcode'] == CommandOpcode.stop_all.value:
                    self.motors.StopAllMotors()

            except Exception as e:
                pass

    def MotorHandler(self, event):
        if len(event) < 2:
            print("motor arg missing")
            return
        speed = event["value"]
        motor = RobotMotor(event["motor"])

        if self.isFlip:
            if motor == RobotMotor.Turn1:
                motor = RobotMotor.Turn2
            elif motor == RobotMotor.Turn2:
                motor = RobotMotor.Turn1
            elif motor == RobotMotor.Drive1 or motor == RobotMotor.Drive2:
                speed = -speed

        if motor == RobotMotor.Turn1:
            motor = RobotMotor.Turn2
        elif motor == RobotMotor.Turn2:
            motor = RobotMotor.Turn1
        elif motor == RobotMotor.Elev1:
            motor = RobotMotor.Joint1

        elif motor == RobotMotor.Joint1:
            motor = RobotMotor.Elev1

        if speed == 0:
            self.motors.MotorStop(motor)
        else:
            if motor == RobotMotor.Drive1:
                self.motors.MotorRun(motor, speed)
                self.motors.MotorRun(RobotMotor.Drive2, speed)
            else:
                self.motors.MotorRun(motor, speed)

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
                    self.motors.MotorStop(RobotMotor.Pump1)
                elif self.activePump == 2:
                    self.motors.MotorStop(RobotMotor.Pump2)
                elif self.activePump == 3:
                    self.motors.MotorStop(RobotMotor.Pump3)
        self.telemetryChannel.send_message(json.dumps(event))

    def CalibrationHandler(self, event):
        self.offset1 = self.angle1
        self.offset2 = self.angle2

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
            if A2D_EXISTS:
                # this function take time i2c a2d issue to solve
                self.motors.MotorTestCurrentOverload(self.a2d.values)

    def TelemetricInfoSend(self):
        if IMU1_EXIST:
            self.angle1 = self.imu_1.prevAngle[0]
        else:
            self.angle1 = [0.0, 0.0, 0.0]
        if IMU2_EXIST:
            self.angle2 = self.imu_2.prevAngle[0]
        else:
            self.angle2 = [0.0, 0.0, 0.0]

        info = {
            "opcode": CommandOpcode.telemetric.name,
            "imu-1": np.subtract(np.array(self.angle1), np.array(self.offset1)).tolist(),
            "imu-2": np.subtract(np.array(self.angle2), np.array(self.offset2)).tolist(),
            "drive1": self.a2d.values[1],
            "drive2": self.a2d.values[2],
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

        if self.telemetryChannel is not None:
            self.telemetryChannel.send_message(json.dumps(info))


if __name__ == "__main__":
    obj = RobotMain()
    obj.motors.MotorRun(RobotMotor.Pump3, 10)
    obj.motors.MotorStop(RobotMotor.Pump3)
    time.sleep(1000000)
