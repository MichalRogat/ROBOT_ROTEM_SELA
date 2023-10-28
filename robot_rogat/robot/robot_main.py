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



KEEP_ALIVE_TIMEOUT_SEC = 1.0
A2D_EXISTS = False
IMU1_EXIST = True
IMU2_EXIST = False

RC = "REMOTE"  # RC=Remote Control

class RobotMain():
    def __init__(self) -> None:
        self.comm_thread = threading.Thread(target=self.CommRxHandle)
        self.rx_q = queue.Queue()
        self.tx_q = queue.Queue()
        if RC == "LOCAL":
            self.ps4Conroller = ps4_controller.RobotPS4(self.rx_q, interface="/dev/input/js0", connecting_using_ds4drv=False)
        else:
            self.comm = robot_remote_control.RobotRemoteControl(self.rx_q)
        
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


    def MotorHandler(self,event):
        if len(event)< 3:
            print("motor arg missing")
            return
        speed = event["value"]
        motor = RobotMotor(event["motor"])
        dir = event["dir"]
        print(f"{motor} dir:{dir} speed:{speed}")
        if speed == 0:
            self.motors.MotorStop(motor)
        else:           
            self.motors.MotorRun(motor,dir,speed)
        
    def KeepAliveHandler(self):
        self.last_keep_alive = datetime.datetime.now()
        self.motors.disable_motors = False
        self.TelemetricInfoSend()

    def RobotMain(self):
        while True:
            delta = datetime.datetime.now() - self.last_keep_alive
            if delta.total_seconds() >= KEEP_ALIVE_TIMEOUT_SEC:
                self.motors.StopAllMotors()

            #read current of motors
            if A2D_EXISTS:
                pass
                #self.motors.MotorTestCurrentOverload() #this function take time i2c a2d issue to solve

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
            "drive1-CS" : self.motors.motor_current[RobotMotor.Drive1.value],
            "drive1-OC" : self.motors.over_current[RobotMotor.Drive1.value],
            "drive2-CS" : self.motors.motor_current[RobotMotor.Drive2.value],
            "drive2-OC" : self.motors.over_current[RobotMotor.Drive2.value],
            "turn1-CS"  : self.motors.motor_current[RobotMotor.Turn1.value],
            "turn1-OC"  : self.motors.over_current[RobotMotor.Turn1.value],
            "turn2-CS"  : self.motors.motor_current[RobotMotor.Turn2.value],
            "turn2-OC"  : self.motors.over_current[RobotMotor.Turn2.value],
            "elev-CS"   : self.motors.motor_current[RobotMotor.Elev1.value],
            "elev-OC"   : self.motors.over_current[RobotMotor.Elev1.value],
            "joint-CS"  : self.motors.motor_current[RobotMotor.Joint1.value],
            "joint-OC"  : self.motors.over_current[RobotMotor.Joint1.value],
            "FullTank1" : 4096,
            "FullTank2" : 4096,
            "FullTank3" : 4096,
            "Spare1"    : 4096,
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
            "Camera-S4" : True
        }
        self.comm.Transmit(info)

if __name__ == "__main__":
    obj = RobotMain()