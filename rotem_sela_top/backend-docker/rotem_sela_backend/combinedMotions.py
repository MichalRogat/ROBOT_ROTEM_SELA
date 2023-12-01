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

    def genericCombinedMotions(drivers:list, speeds:list):
        for driver, speed in zip(drivers,speeds):
            Entity.IMotor.getByName(driver).MotorRun(speed)