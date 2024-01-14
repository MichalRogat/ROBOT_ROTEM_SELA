from Events import KeyboardEvents
import Entity
import time
import json

UP_FAST = RIGHT_FAST = RUN_FAST = 100
UP_MODERATE = RIGHT_MODERATE = 70
UP_SLOW = RIGHT_SLOW = RUN_SLOW = 30

DOWN_FAST = LEFT_FAST = -100
DOWN_MODERATE = LEFT_MODERATE = -70
DOWN_SLOW = LEFT_SLOW = -30

T1 = T2 = T3 = T4 = 0
E1 = E2 = E3 = E4 = 1

class CombinedMotions():

    def combinedMotionsMotorRun(drivers:list, speeds:list):
        with open('../../../../entitiesFlipping.json', 'r') as file:
            json_data = json.load(file)
        reverse_dir_list = json_data.get("reverseDir", [])
        
        for driver, speed in zip(drivers,speeds):
            if (driver in reverse_dir_list):
                 speed = -speed
            Entity.IMotor.getByName(driver).MotorRun(speed)

    def combinedMotionsMotorStop(drivers:list):
        for driver in drivers:
            Entity.IMotor.getByName(driver).stopMotor()