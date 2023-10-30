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
    L1= 4
    R1= 5
    L2= 6
    R2= 7
    share=8
    options= 9
    left_stick_click= 10
    up_arrow= 12
    right_stick_click= 11
    down_arrow= 13
    left_arrow= 14
    right_arrow= 15
    touchpad= 17

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
        self.prev_axis_val = [0]*8
        self.controller.init()
        self.x_pressed = False
        self.triangle_pressed = False
        self.square_pressed = False
        self.l3_pressed = False
        self.r3_pressed = False
        self.circle_pressed = False
        self.r1_pressed = False
        self.L1_pressed = False
        self.L2_pressed = False
        self.R2_pressed = False
        self.elev_up_pressed = False
        self.elev_down_pressed = False
        self.joint_open_shrink_pressed = False
        self.joint_shrink_pressed = False

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
        drive_mode = "f"
        while True:
            events = pygame.event.get()
            for event in events:
                if event.type == pygame.JOYAXISMOTION:
                    speed = int(event.value * 100)
                    if event.axis == ps4_joystic.left_stick_x.value:
                        if drive_mode == "r":
                            motor = RobotMotor.Turn1
                        else:
                            motor = RobotMotor.Turn2
                        self.put_in_q(motor, speed)
                    if event.axis == ps4_joystic.left_stick_y.value:
                        pass
                    if event.axis == ps4_joystic.right_stick_x.value:
                        if drive_mode == "r":
                            motor = RobotMotor.Turn2
                        else:
                            motor = RobotMotor.Turn1
                        self.put_in_q(motor, speed)
                    if event.axis == ps4_joystic.right_stick_y.value:
                        speed=-speed
                        if drive_mode == "r":
                            speed = -speed
                        self.put_in_q(RobotMotor.Drive1,speed)
                    if event.axis == ps4_joystic.left_trigger.value:
                        pass
                    if event.axis == ps4_joystic.right_trigger.value:
                        pass

                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == ps4_buttons.x.value:
                        self.x_pressed = True
                    #active pumping
                    if event.button == ps4_buttons.L2.value:
                        self.L2_pressed = True
                        if self.R2_pressed == True:
                            self.put_in_q_pump(activePumping=1)   
                    if event.button == ps4_buttons.R2.value:
                        self.R2_pressed = True
                        if self.L2_pressed == True:
                            self.put_in_q_pump(activePumping=1)   

                    if event.button == ps4_buttons.circle.value:
                        self.circle_pressed = True
                        self.put_in_q_camera(isToggle=1)
                    if event.button == ps4_buttons.R1.value:
                        self.r1_pressed = True
                    if event.button == ps4_buttons.L1.value:
                        self.put_in_q_pump(togglePumps=1)
                        self.L1_pressed = True
                    if event.button == ps4_buttons.triangle.value:
                        drive_mode = "f" if drive_mode == "r" else "r" #flip driving direction
                        self.put_in_q_camera(isFlip=1)
                        self.triangle_pressed = True
                    if event.button == ps4_buttons.square.value:
                        self.put_in_q_camera(lightLevel=1)
                        self.square_pressed = True
                    #elevation
                    if event.button == ps4_buttons.up_arrow.value:
                        self.put_in_q(RobotMotor.Elev1, 50)
                        self.elev_up_pressed = True
                    if event.button == ps4_buttons.down_arrow.value:
                        self.put_in_q(RobotMotor.Elev1, -50)
                        self.elev_down_pressed = True
                    #joint shrink
                    if event.button == ps4_buttons.right_arrow.value:
                        self.put_in_q(RobotMotor.Joint1, 50)
                        self.joint_open_shrink_pressed = True
                    if event.button == ps4_buttons.left_arrow.value:
                        self.put_in_q(RobotMotor.Joint1, -50)
                        self.joint_shrink_pressed = True

                if event.type == pygame.JOYBUTTONUP:
                    if event.button == ps4_buttons.x.value:
                        self.x_pressed = False
                    if event.button == ps4_buttons.triangle.value:
                        self.triangle_pressed = False
                    if event.button == ps4_buttons.circle.value:
                        self.circle_pressed = False
                    if event.button == ps4_buttons.square.value:
                        self.square_pressed = False
                    if event.button == ps4_buttons.R1.value:
                        self.r1_pressed = False
                    if event.button == ps4_buttons.L1.value:
                        self.L1_pressed = False
                    #stop pumping
                    if event.button == ps4_buttons.L2.value:
                        self.L2_pressed = False
                        self.put_in_q_pump(activePumping=0)   
                    if event.button == ps4_buttons.R2.value:
                        self.R2_pressed = False
                        self.put_in_q_pump(activePumping=1)   
                    #elevation
                    if event.button == ps4_buttons.up_arrow.value:
                        self.put_in_q(RobotMotor.Elev1, 0)
                        self.elev_up_pressed = False
                    if event.button == ps4_buttons.down_arrow.value:
                        self.put_in_q(RobotMotor.Elev1, 0)
                        self.elev_down_pressed = False
                    #joint shrink release
                    if event.button == ps4_buttons.right_arrow.value:
                        self.put_in_q(RobotMotor.Joint1, 0)
                        self.joint_open_shrink_pressed = False
                    if event.button == ps4_buttons.left_arrow.value:
                        self.put_in_q(RobotMotor.Joint1, 0)
                        self.joint_shrink_pressed = False
        
    def put_in_q(self,motor,speed):
        if abs(speed)<5:
            speed = 0

        #prevent resend when stick is released
        if speed == 0 and self.prev_axis_val[motor.value] == 0:
            self.prev_axis_val[motor.value] = speed
            return
        
        #filter out small changes to prevent network overload
        if abs(speed - self.prev_axis_val[motor.value]) < 5:
            return

        #detect stick is released fast
        if speed < 10 and (self.prev_axis_val[motor.value] - speed) > 20:
            speed =0

        self.prev_axis_val[motor.value] = speed
        str = {"opcode":CommandOpcode.motor,"motor": motor,"value":speed}
        print(str)
        self.rc_q.put(str)

    # isFlip = 1 means we changed driving direction between front and back
    # isToggle = 1 means we change between right and left cameras in current direction
    #lightLevel = 1 means we want to toggle light levels (3 degrees on robot side: strong, weak, off)
    def put_in_q_camera(self, isFlip=0, isToggle=0, lightLevel=0): 
        str = {"opcode":CommandOpcode.camera,"isFlip":isFlip,"isToggle":isToggle,"lightLevel":lightLevel}
        print(str)
        self.rc_q.put(str)

    # togglePumps = 1 means we toggle between pumps
    # activePumping = 1 means we are actively pumping
    def put_in_q_pump(self, togglePumps=0, activePumping=0): 
        str = {"opcode":CommandOpcode.pump,"togglePumps":togglePumps,"activePumping":activePumping}
        print(str)
        self.rc_q.put(str)

    
if __name__ == "__main__":
    dummy = queue.Queue()
    ps4 = PS4Controller(dummy)
    ps4.listen()