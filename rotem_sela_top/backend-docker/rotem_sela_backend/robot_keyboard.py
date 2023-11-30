import keyboard
from keyboard._keyboard_event import KEY_DOWN, KEY_UP
from pwm_rpi import RobotMotor
import queue
import time
 
class RobotKeyboard():
    def __init__(self, key_queue):
        self.ps_queue = key_queue
        keyboard.hook(lambda e: self.on_action(e))
        
    def on_action(self, event):
        if event.event_type == KEY_DOWN:
            self.on_press(event.name)
        elif event.event_type == KEY_UP:
            self.on_release(event.name)
        
    def listen(self):
        self.speed = 0 
        while True:
            time.sleep(0.1) 
                       
    def on_press(self,key):
        if self.speed< 32000:
            self.speed += 1000
        value = self.speed
        if key == 'q':
            self.ps_queue.put({"motor":RobotMotor.Drive1,"dir":"forward","value":int(abs(value)/325)})
        if key == 'a':
            self.ps_queue.put({"motor":RobotMotor.Drive1,"dir":"reverse","value":int(abs(value)/325)})
        if key == 'w':
            self.ps_queue.put({"motor":RobotMotor.Elev1,"dir":"forward","value":int(abs(value)/325)})
        if key == 's':
            self.ps_queue.put({"motor":RobotMotor.Elev1,"dir":"reverse","value":int(abs(value)/325)})
        if key == 'e':
            self.ps_queue.put({"motor":RobotMotor.Joint1,"dir":"forward","value":int(abs(value)/325)})
        if key == 'd':
            self.ps_queue.put({"motor":RobotMotor.Joint1,"dir":"reverse","value":int(abs(value)/325)})
        if key == 'r':
            self.ps_queue.put({"motor":RobotMotor.Drive2,"dir":"forward","value":int(abs(value)/325)})
        if key == 'f':
            self.ps_queue.put({"motor":RobotMotor.Drive2,"dir":"reverse","value":int(abs(value)/325)})
    
    def on_release(self,key):
        self.speed = 0
        self.ps_queue.put({"motor":RobotMotor.Drive1,"dir":"forward","value": 0})
    
if __name__ == "__main__":
    dummy = queue.Queue()
    controller = RobotKeyboard(dummy)
    controller.listen()
    