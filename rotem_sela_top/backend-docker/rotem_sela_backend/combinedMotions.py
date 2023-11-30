from Events import KeyboardEvents
import Entity

PHIGH = 100
PMEDIUM = 70
PLOW = 30

MHIGH = -100
MMEDIUM = -70
MLOW = -30

JOINT1 = 0
JOINT2 = 1
JOING3 = 2
JOINT4 = 3
T1 = T2 = T3 = T4 = 0
E1 = E2 = E3 = E4 = 1

class CombinedMotions():
    
    def switchEvents(joints:[[Entity.Driver]], e):
        if e == KeyboardEvents.K_w:
            joints[JOINT4][E4].MotorRun(PHIGH)
            joints[JOING3][E3].MotorRun(MHIGH)
        
        elif e == KeyboardEvents.K_q:
            raise NotImplemented        
        
        elif e == KeyboardEvents.K_a:
            raise NotImplemented
        
        elif e == KeyboardEvents.K_s:
            raise NotImplemented
        
        elif e == KeyboardEvents.K_e:
            raise NotImplemented        
        
        elif e == KeyboardEvents.K_d:
            raise NotImplemented
        
        elif e == KeyboardEvents.K_z:
            raise NotImplemented
        
        elif e == KeyboardEvents.K_x:
            raise NotImplemented