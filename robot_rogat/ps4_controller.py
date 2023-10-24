from pyPS4Controller.controller import Controller
import queue
from pwm_rpi import RobotMotor
    

class RobotPS4(Controller):
    def __init__(self, ps_queue, **kwargs):
        Controller.__init__(self, **kwargs)
        self.ps_queue = ps_queue
    
    def on_x_press(self):
        self.ps_queue.put({"motor":RobotMotor.Func1,"value":1})
     
    def on_x_release(self):
        self.ps_queue.put({"motor":RobotMotor.Func1,"value":0})
    
    def on_circle_press(self):
        self.ps_queue.put({"motor":RobotMotor.Func2,"value":1})
     
    def on_circle_release(self):
        self.ps_queue.put({"motor":RobotMotor.Func2,"value":0})
    
    def on_square_press(self):
        self.ps_queue.put({"motor":RobotMotor.Func3,"value":1})
     
    def on_square_release(self):
        self.ps_queue.put({"motor":RobotMotor.Func3,"value":0})

    def on_R3_right(self, value):
        self.ps_queue.put({"motor":RobotMotor.Head,"dir":"forward","value":int(abs(value)/325)})
    def on_R3_left(self, value):
        self.ps_queue.put({"motor":RobotMotor.Head,"dir":"reverse","value":int(abs(value)/325)})
        
    def on_R3_up(self, value):
        self.ps_queue.put({"motor":RobotMotor.HeadTurn,"dir":"forward","value":int(abs(value)/325)})
    def on_R3_down(self, value):
        self.ps_queue.put({"motor":RobotMotor.HeadTurn,"dir":"reverse","value":int(abs(value)/325)})
        
    def on_L3_right(self, value):
        self.ps_queue.put({"motor":RobotMotor.Tail,"dir":"forward","value":int(abs(value)/325)})
    def on_L3_left(self, value):
        self.ps_queue.put({"motor":RobotMotor.Tail,"dir":"reverse","value":int(abs(value)/325)})
        
    def on_L3_up(self, value):
        self.ps_queue.put({"motor":RobotMotor.TailTurn,"dir":"forward","value":int(abs(value)/325)})
    def on_L3_down(self, value):
        self.ps_queue.put({"motor":RobotMotor.TailTurn,"dir":"reverse","value":int(abs(value)/325)})
    
   

        
    

if __name__ == "__main__":
    dummy = queue.Queue()
    controller = RobotPS4(dummy, interface="/dev/input/js0", connecting_using_ds4drv=False,)
    controller.listen(timeout=60)