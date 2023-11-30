from Events import KeyboardEvents
import Entity
import time

UP_FAST = RIGHT_FAST = RUN_FAST = 100
UP_MODERATE = RIGHT_MODERATE = 70
UP_SLOW = RIGHT_SLOW = RUN_SLOW = 30

DOWN_FAST = LEFT_FAST = -100
DOWN_MODERATE = LEFT_MODERATE = -70
DOWN_SLOW = LEFT_SLOW = -30

T1 = T2 = T3 = T4 = 0
E1 = E2 = E3 = E4 = 1

class CombinedMotions():

    def genericCombinedMotions(drivers:list[tuple]):
        for driver, speed in drivers:
            driver.MotorRun(speed)
    
    def switchRunEvents(joints:[[Entity.Driver]], e, motors:Entity.Driver):
        if e == KeyboardEvents.K_w:
            joints[E4][E4].MotorRun(UP_FAST)
            joints[E3][E3].MotorRun(DOWN_FAST)
        
        elif e == KeyboardEvents.K_q:
            joints[E1][E1].MotorRun(UP_FAST)
            joints[E2][E2].MotorRun(DOWN_FAST)
        
        elif e == KeyboardEvents.K_a:
            joints[E1][E1].MotorRun(DOWN_FAST)
            joints[E2][E2].MotorRun(UP_FAST)
        
        elif e == KeyboardEvents.K_s:
            joints[E4][E4].MotorRun(DOWN_FAST)
            joints[E3][E3].MotorRun(UP_FAST)
        
        elif e == KeyboardEvents.K_e:
            joints[E2][E2].MotorRun(DOWN_FAST)
            joints[E1][E1].MotorRun(UP_SLOW)
        
        elif e == KeyboardEvents.K_d:
            joints[E1][E1].MotorRun(UP_SLOW)
            joints[E2][E2].MotorRun(DOWN_FAST)
        
        elif e == KeyboardEvents.K_z:
            joints[T1][T1].MotorRun(LEFT_FAST)
            motors.trailer1.driver1(RUN_SLOW)
            joints[T2][T2].MotorRun(RIGHT_FAST)

        elif e == KeyboardEvents.K_x:
            joints[T1][T1].MotorRun(LEFT_SLOW)
            joints[E1][E1].MotorRun(DOWN_SLOW)
            joints[E2][E2].MotorRun(UP_FAST)
            joints[T2][T2].MotorRun(RIGHT_FAST)
            joints[E3][E3].MotorRun(DOWN_FAST)
            joints[T3][T3].MotorRun(LEFT_FAST)            

    def switchStopEvents(joints:[[Entity.Driver]], e, motors:Entity.Driver):
        if e == KeyboardEvents.K_w:
            joints[E4][E4].stopMotor()
            joints[E3][E3].stopMotor()
        
        elif e == KeyboardEvents.K_q:
            joints[E1][E1].stopMotor()
            joints[E2][E2].stopMotor()
        
        elif e == KeyboardEvents.K_a:
            joints[E1][E1].stopMotor()
            joints[E2][E2].stopMotor()
        
        elif e == KeyboardEvents.K_s:
            joints[E4][E4].stopMotor()
            joints[E3][E3].stopMotor()
        
        elif e == KeyboardEvents.K_e:
            joints[E2][E2].stopMotor()
            joints[E1][E1].stopMotor()
        
        elif e == KeyboardEvents.K_d:
            joints[E1][E1].stopMotor()
            joints[E2][E2].stopMotor()
        
        elif e == KeyboardEvents.K_z:
            joints[T1][T1].stopMotor()
            motors.trailer1.driver1(RUN_SLOW)
            joints[T2][T2].stopMotor()

        elif e == KeyboardEvents.K_x:
            joints[T1][T1].stopMotor()
            joints[E1][E1].stopMotor()
            joints[E2][E2].stopMotor()
            joints[T2][T2].stopMotor()
            joints[E3][E3].stopMotor()
            joints[T3][T3].stopMotor()  