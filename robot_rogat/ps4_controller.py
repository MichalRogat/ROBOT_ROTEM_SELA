from pyPS4Controller.controller import Controller
import queue
from pwm_rpi import RobotMotor
    

class RobotPS4(Controller):
    def __init__(self, ps_queue, **kwargs):
        Controller.__init__(self, **kwargs)
        self.ps_queue = ps_queue
        self.x_pressed = False
        self.l3_pressed = False
        self.r3_pressed = False
        self.circle_pressed = False
        self.r1_pressed = False
        self.l1_pressed = "forward"
    
    def on_L3_press(self):
        self.l3_pressed = True
        
    def on_L3_release(self):
        self.l3_pressed = False
    
    def on_R1_press(self):
        self.r1_pressed = True
        
    def on_R1_release(self):
        self.r1_pressed = False
        
    def on_R3_press(self):
        self.r3_pressed = True
        
    def on_R3_release(self):
        self.r3_pressed = False
    
    def on_x_press(self):
        self.x_pressed = True
    
    def on_x_release(self):
        self.circle_pressed = False
        
    def on_x_press(self):
        self.x_pressed = True
    
    def on_x_release(self):
        self.x_pressed = False
    
    def on_R3_right(self, value):
        self.ps_queue.put({"motor":RobotMotor.Turn1,"dir":"forward","value":int(abs(value)/325)})
    def on_R3_left(self, value):
        self.ps_queue.put({"motor":RobotMotor.Turn1,"dir":"reverse","value":int(abs(value)/325)})
        
    def on_R3_up(self, value):
            self.ps_queue.put({"motor":RobotMotor.Drive1,"dir":"forward","value":int(abs(value)/325)})
        
    def on_R3_down(self, value):
            self.ps_queue.put({"motor":RobotMotor.Drive1,"dir":"reverse","value":int(abs(value)/325)})
            
    def on_L3_right(self, value):
        self.ps_queue.put({"motor":RobotMotor.Turn2,"dir":"forward","value":int(abs(value)/325)})
        
    def on_L3_left(self, value):
        self.ps_queue.put({"motor":RobotMotor.Turn2,"dir":"reverse","value":int(abs(value)/325)})
        
    def on_L3_up(self, value):
        if self.r1_pressed:
            self.ps_queue.put({"motor":RobotMotor.Joint1,"dir":"forward","value":int(abs(value)/325)})
            return
        if self.x_pressed:
            self.ps_queue.put({"motor":RobotMotor.Joint2,"dir":"forward","value":int(abs(value)/325)})
            return
        self.ps_queue.put({"motor":RobotMotor.Drive2,"dir":"forward","value":int(abs(value)/325)})
        
    def on_L3_down(self, value):
        if self.x_pressed:
            self.ps_queue.put({"motor":RobotMotor.Joint1,"dir":"reverse","value":int(abs(value)/325)})
            return
        if self.r1_pressed:
            self.ps_queue.put({"motor":RobotMotor.Joint2,"dir":"reverse","value":int(abs(value)/325)})
            return
        self.ps_queue.put({"motor":RobotMotor.Drive2,"dir":"reverse","value":int(abs(value)/325)})    
    def on_R2_press(self, value):
        if self.x_pressed:
            dir = "forward"
        else:
            dir = "reverse"
        self.ps_queue.put({"motor":RobotMotor.Elev1,"dir": dir,"value":int(abs(value)/325)})
    
    def on_R2_release(self):
        self.ps_queue.put({"motor":RobotMotor.Elev1,"dir": self.x_pressed,"value":0})
    
    def on_L2_press(self, value):
        if self.x_pressed:
            dir = "forward"
        else:
            dir = "reverse"
        self.ps_queue.put({"motor":RobotMotor.Elev2,"dir": dir,"value":int(abs(value)/325)})
    
    def on_L2_release(self):
        self.ps_queue.put({"motor":RobotMotor.Elev2,"dir": self.x_pressed,"value":0})

    def on_up_arrow_press(self):
        if self.x_pressed:
            self.ps_queue.put({"motor":RobotMotor.Pump1,"dir":"forward","value": 1})
        else:
            self.ps_queue.put({"motor":RobotMotor.Pump1,"dir":"reverse","value": 1})
    
    def on_down_arrow_press(self):
        if self.x_pressed:
            self.ps_queue.put({"motor":RobotMotor.Pump2,"dir":"forward","value": 1})
        else:
            self.ps_queue.put({"motor":RobotMotor.Pump2,"dir":"reverse","value": 1})
            
    def on_right_arrow_press(self):
        if self.x_pressed:
            self.ps_queue.put({"motor":RobotMotor.Pump3,"dir":"forward","value": 1})
        else:
            self.ps_queue.put({"motor":RobotMotor.Pump3,"dir":"reverse","value": 1})
            
    def on_left_arrow_press(self):
        if self.x_pressed:
            self.ps_queue.put({"motor":RobotMotor.Pump4,"dir":"forward","value": 1})
        else:
            self.ps_queue.put({"motor":RobotMotor.Pump4,"dir":"reverse","value": 1})
            
    def on_left_right_arrow_release(self):
        self.ps_queue.put({"motor":RobotMotor.Pump1,"dir":"reverse","value": 0})
    
    def on_up_down_arrow_release(self):
        self.ps_queue.put({"motor":RobotMotor.Pump1,"dir":"reverse","value": 0})
    
if __name__ == "__main__":
    dummy = queue.Queue()
    controller = RobotPS4(dummy, interface="/dev/input/js0", connecting_using_ds4drv=False,)
    controller.listen(timeout=60)