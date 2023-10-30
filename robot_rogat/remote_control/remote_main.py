import sys 
import threading
import time
import queue
import ps4_usb
import robot_comm
from pwm_rpi import RobotMotor
from enum import Enum

KEEP_ALIVE_PERIOD = 0.25

class CommandOpcode(Enum):
    motor = 1
    keep_alive = 2
    camera = 3
    telemetry = 4
    pump = 5

class RobotRemote():
    def __init__(self) -> None:

        self.rx_q = queue.Queue()
        self.comm = robot_comm.RobotComm(self.rx_q)
        self.remote = ps4_usb.PS4Controller(self.rx_q)
        if self.remote.controller == None:
            return
        self.comm_thread = threading.Thread(target=self.CommRxHandler)
        self.ps4_thread  = threading.Thread(target=self.Ps4Handler)
        self.main_thread = threading.Thread(target=self.RobotMain)
        self.ka_thread = threading.Thread(target=self.KeepAlive)

        self.comm_thread.start()
        self.main_thread.start()
        self.ps4_thread.start()
        self.ka_thread.start()
    
    #this task is repsonsilbe for receiving messages from the ps4 controller
    def Ps4Handler(self):
        self.remote.listen()

    #this task is responsible for receiving messages from the robot
    def CommRxHandler(self):
        self.comm.RxHandler()
    
    #this task is responsible for sending keep alive messages to the robot
    def KeepAlive(self):
        self.comm.Transmit({"opcode":CommandOpcode.keep_alive}) 
        threading.Timer(KEEP_ALIVE_PERIOD, self.KeepAlive).start()
    
    #this task is responsible for sending messages to the robot
    def RobotMain(self):
        while True:
            event = self.rx_q.get()          
            self.comm.Transmit(event)

                
if __name__ == "__main__":
    obj = RobotRemote()