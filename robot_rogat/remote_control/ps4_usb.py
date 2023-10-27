import pygame
import queue
import time
from enum import Enum
from pwm_rpi import RobotMotor
from remote_main import CommandOpcode

class ps4_joystic(Enum):
    left_stick_x = 0
    left_stick_y = 1
    right_stick_x = 2
    right_stick_y = 3
    left_trigger = 4
    right_trigger = 5
    dpad_x = 6
    dpad_y = 7

class ps4_buttons(Enum):
    x = 0
    circle= 1
    square= 2
    triangle= 3
    share= 4
    PS= 5
    options= 6
    left_stick_click= 7
    right_stick_click= 8
    L1= 9
    R1= 10
    up_arrow= 11
    down_arrow= 12
    left_arrow= 13
    right_arrow= 14
    touchpad= 15

class PS4Controller(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""
    controller = None
    axis_data = None
    button_data = None
    hat_data = None

    def __init__(self, rc_q):
        """Initialize the joystick components"""
        
        pygame.init()
        pygame.joystick.init()
        self.rc_q = rc_q
        try:
            self.controller = pygame.joystick.Joystick(0)
        except pygame.error:
            print("Joystick not found.")
            return None
        self.controller.init()
        self.x_pressed = False
        self.l3_pressed = False
        self.r3_pressed = False
        self.circle_pressed = False
        self.r1_pressed = False
        self.l1_pressed = "forward"

    def listen(self):
        """Listen for events to happen"""
        
        if not self.axis_data:
            self.axis_data = {}

        if not self.button_data:
            self.button_data = {}
            for i in range(self.controller.get_numbuttons()):
                self.button_data[i] = False

        if not self.hat_data:
            self.hat_data = {}
            for i in range(self.controller.get_numhats()):
                self.hat_data[i] = (0, 0)

        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYAXISMOTION:
                    if event.axis == ps4_joystic.left_stick_x.value:
                        speed = int(event.value * 100)
                        if speed > 0:
                            dir = "forward"
                        else:
                            dir = "reverse"
                        self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Turn2,"dir":dir ,"value":abs(speed)})
                    if event.axis == ps4_joystic.left_stick_y.value:
                        speed = int(event.value * 100)
                        if speed < 0:
                            dir = "forward"
                        else:
                            dir = "reverse"
                        if self.x_pressed:
                            motor = RobotMotor.Joint1
                        elif self.r1_pressed:
                            motor = RobotMotor.Joint2
                        else:
                            motor = RobotMotor.Drive2
                        self.rc_q.put({"opcode":CommandOpcode.motor,"motor":motor,"dir":dir ,"value":abs(speed)})
                    if event.axis == ps4_joystic.right_stick_x.value:
                        speed = int(event.value * 100)
                        if speed > 0:
                            dir = "forward"
                        else:
                            dir = "reverse"
                        self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Turn1,"dir":dir ,"value":abs(speed)})
                    if event.axis == ps4_joystic.right_stick_y.value:
                        speed = int(event.value * 100)
                        if speed < 0:
                            dir = "forward"
                        else:
                            dir = "reverse"
                        self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Drive1,"dir":dir ,"value":abs(speed)})
                    if event.axis == ps4_joystic.left_trigger.value:
                        speed = int(event.value * 100)
                        if self.x_pressed:
                            dir = "forward"
                        else:
                            dir = "reverse"
                        self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Elev2,"dir": dir,"value":speed})
                    if event.axis == ps4_joystic.right_trigger.value:
                        speed = int(event.value * 100)
                        if self.x_pressed:
                            dir = "forward"
                        else:
                            dir = "reverse"
                        self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Elev1,"dir": dir,"value":speed})
                
                # elif event.type == pygame.JOYBALLMOTION:
                #     print(event.dict, event.joy, event.ball, event.rel)
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == ps4_buttons.x.value:
                        self.x_pressed = True
                    if event.button == ps4_buttons.circle.value:
                        self.circle_pressed = True
                    if event.button == ps4_buttons.R1.value:
                        self.r1_pressed = True
                    if event.button == ps4_buttons.L1.value:
                        self.l1_pressed = True
                    if event.button == ps4_buttons.up_arrow.value:
                        if self.x_pressed:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump1,"dir":"forward","value": 1})
                        else:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump1,"dir":"reverse","value": 1})
                    if event.button == ps4_buttons.down_arrow.value:
                        if self.x_pressed:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump2,"dir":"forward","value": 1})
                        else:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump2,"dir":"reverse","value": 1})
                    if event.button == ps4_buttons.right_arrow.value:
                        if self.x_pressed:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump3,"dir":"forward","value": 1})
                        else:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump3,"dir":"reverse","value": 1})
                    if event.button == ps4_buttons.left_arrow.value:
                        if self.x_pressed:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump4,"dir":"forward","value": 1})
                        else:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump4,"dir":"reverse","value": 1})   
                elif event.type == pygame.JOYBUTTONUP:
                    if event.button == ps4_buttons.x.value:
                        self.x_pressed = False
                    if event.button == ps4_buttons.circle.value:
                        self.circle_pressed = False
                    if event.button == ps4_buttons.R1.value:
                        self.r1_pressed = False
                    if event.button == ps4_buttons.L1.value:
                        self.l1_pressed = False
                    if event.button == ps4_buttons.up_arrow.value:
                        if self.x_pressed:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump1,"dir":"forward","value": 0})
                        else:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump1,"dir":"reverse","value": 0})
                    if event.button == ps4_buttons.down_arrow.value:
                        if self.x_pressed:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump2,"dir":"forward","value": 0})
                        else:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump2,"dir":"reverse","value": 0})
                    if event.button == ps4_buttons.right_arrow.value:
                        if self.x_pressed:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump3,"dir":"forward","value": 0})
                        else:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump3,"dir":"reverse","value": 0})
                    if event.button == ps4_buttons.left_arrow.value:
                        if self.x_pressed:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump4,"dir":"forward","value": 0})
                        else:
                            self.rc_q.put({"opcode":CommandOpcode.motor,"motor":RobotMotor.Pump4,"dir":"reverse","value": 0})        
               
            time.sleep(0.1)


if __name__ == "__main__":
    dummy = queue.Queue()
    ps4 = PS4Controller(dummy)
    ps4.listen()