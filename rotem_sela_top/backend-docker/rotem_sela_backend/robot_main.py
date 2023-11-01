import threading
import time
import queue
import logging
import os
import datetime
from pwm_rpi import RobotMotor,MotorDriver
import ps4_controller
import robot_remote_control
from robot_remote_control import CommandOpcode
from enum import Enum
from minimu import MinIMU_v5_pi
import serial_a2d


KEEP_ALIVE_TIMEOUT_SEC = 1.0
A2D_EXISTS = True
IMU1_EXIST = True
IMU2_EXIST = False

RC = "REMOTE"  # RC=Remote Control

class RobotMain():

    telemetryChannel = None
    
    def __init__(self) -> None:
        print(f"Start robot service {RC}")
        self.activePump = 1
        self.isPumpingNow = 0
        self.comm_thread = threading.Thread(target=self.CommRxHandle)
        self.rx_q = queue.Queue()
        self.tx_q = queue.Queue()
        if RC == "LOCAL":
            self.ps4Conroller = ps4_controller.RobotPS4(self.rx_q, interface="/dev/input/js0", connecting_using_ds4drv=False)
        else:
            print(f"Start remote control service")
            self.comm = robot_remote_control.RobotRemoteControl(self.rx_q)
        
        self.a2d = serial_a2d.SerialA2D()
        self.a2d_thread = threading.Thread(target=self.A2dHandler)
        self.i2c_lock = threading.Lock()
        self.message_handler_thread = threading.Thread(target=self.RobotMessageHandler)
        self.main_thread = threading.Thread(target=self.RobotMain)
        self.last_keep_alive = datetime.datetime.now()
        self.motors = MotorDriver()
        if IMU1_EXIST:
            self.imu_1 = MinIMU_v5_pi(0, self.i2c_lock)
            self.imu_1.trackAngle()

        if IMU2_EXIST:
            self.imu_2 = MinIMU_v5_pi(1, self.i2c_lock)
            self.imu_2.trackAngle()

        self.comm_thread.start()
        self.message_handler_thread.start()
        self.main_thread.start()
        self.a2d_thread.start()
        self.motors.StopAllMotors()
    
    def setTelemetryChannel(self, channel):
        self.telemetryChannel = channel

    def A2dHandler(self):
        self.a2d.listen()
       
    def CommRxHandle(self):
        if RC == "LOCAL":
            self.ps4Conroller.listen()
        else:
            while True:
                self.comm.start()
        
    def RobotMessageHandler(self):
        while True:
            event = self.rx_q.get(0.5)
            if event["opcode"] == CommandOpcode.motor.value:
                self.MotorHandler(event)
            if event["opcode"] == CommandOpcode.keep_alive.value:
                self.KeepAliveHandler()
            if event["opcode"] == CommandOpcode.camera.value:
                self.CameraHandler(event)
            if event["opcode"] == CommandOpcode.pump.value:
                self.PumpHandler(event)


    def MotorHandler(self,event):
        if len(event)< 2:
            print("motor arg missing")
            return
        speed = event["value"]
        motor = RobotMotor(event["motor"])
        if speed == 0:
            self.motors.MotorStop(motor)
        else:
            if motor == RobotMotor.Drive1:
                self.motors.MotorRun(motor,speed)
                self.motors.MotorRun(RobotMotor.Drive2,speed)
            else:
                self.motors.MotorRun(motor,speed)
           
        
    def KeepAliveHandler(self):
        self.last_keep_alive = datetime.datetime.now()
        self.motors.disable_motors = False
        self.TelemetricInfoSend()

    def CameraHandler(self, event):
        if len(event) < 2:
            print("camera arg missing")
            return
        isToggle = event["isToggle"]
        isFlip = event["isFlip"]
        lightLevel = event["lightLevel"]
        if(isToggle):
            #toggle cameras (left / right)
            print(f"Toggle cameras")
            pass
        if(isFlip):
            #flip cameras (front / back)
            print(f"Flip cameras")
            pass
        if(lightLevel):
            #toggle light level (off, low, high)
            pass
        
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

            #pump with active pump selected
            #pass
        

    def RobotMain(self):
        while True:
            # print(self.a2d.values)
            delta = datetime.datetime.now() - self.last_keep_alive
            # if delta.total_seconds() >= KEEP_ALIVE_TIMEOUT_SEC:
            #     self.motors.StopAllMotors()

            #read current of motors
            if A2D_EXISTS:
                self.motors.MotorTestCurrentOverload(self.a2d.values) #this function take time i2c a2d issue to solve

    def TelemetricInfoSend(self):
        if IMU1_EXIST:
            angle1=self.imu_1.prevAngle[0]
        else:
            angle1=[0.0,0.0,0.0]
        if IMU2_EXIST:
            angle2=self.imu_2.prevAngle[0]
        else:
            angle2=[0.0,0.0,0.0]
        info = {
            "opcode": CommandOpcode.telemetric,
            "imu-1" : angle1,
            "imu-2" : angle2,
            "drive1" : self.a2d.values[1],
            "drive2" : self.a2d.values[2],
            "elev"   : self.a2d.values[4],
            "turn1"  : self.a2d.values[3],
            "turn2"  : self.a2d.values[5],
            "joint1" : self.a2d.values[6],
            "FullTank1" : self.a2d.values[7],
            "FullTank2" : self.a2d.values[8],
            "FullTank3" : self.a2d.values[9],
            "activePump"    : self.activePump,
            "Spare2"    : 4096,
            "Spare3"    : 4096,
            "Spare4"    : 4096,
            "Spare5"    : 4096,
            "Spare6"    : 4096,
            "Spare7"    : 4096,
            "Camera-F1" : True,
            "Camera-S1" : True,
            "Camera-F2" : True,
            "Camera-S2" : True,
            "Camera-F3" : True,
            "Camera-S3" : True,
            "Camera-F4" : True,
            "Camera-S4" : True,
            
        }

        # print(f"Send telemetry")
        self.telemetryChannel.send_message(str(info))

if __name__ == "__main__":
    obj = RobotMain()
    obj.motors.MotorRun(RobotMotor.Joint1, -90)
    time.sleep(1000000)