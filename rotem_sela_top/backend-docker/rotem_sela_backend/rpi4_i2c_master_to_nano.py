import smbus2
import time

# motor 1
# Set GPIO on pin 2 to high (low to shut down) - digital
# Set PWM on pin 5 to 0
# Set PWM on pin 3 to 50%
# flip direction by fliping pin 5 and 3 role

# motor 2
# Set GPIO on pin 7 to high (low to shut down) - digital
# Set PWM on pin 9 to 50% analog
# Set PWM on pin 6 to 0 analog
# flip direction by fliping pin 6 and 9 role


#motor 3 big driver
# set GPIO 12 to 1 (low 0 to stop) - digital
# set pin 11 pwm 50% (0 to stop) analog
# set GPIO 10 to 1 (low 0 to flip direction) - digital

SET_PWM = 0
STOP_PWM = 1
SET_GPIO = 2
READ_ADC = 3
READ_GPIO = 4

#this way we have the pins of motor 1 in place[1] etc.
motors_pins = [[0, 0, 0], [2, 5, 3], [7, 9, 6], [12, 11, 10]]

# Define I2C address and other parameters
arduino_address = 0x55
start_byte = 0xAA
opcode = 0x00
payload = []
checksum = 0

# Initialize the I2C bus
bus = smbus2.SMBus(1)  # 1 indicates the I2C bus number

# Function to calculate the checksum
def calculate_checksum(packet):
    checksum = 0
    checksum += start_byte
    checksum += opcode
    checksum += (len(packet))

    for data in packet:
        checksum += data
    #print("calculated checksum: ", checksum%255)
    return (checksum%255) 

def startMotor(motor_num):
    global opcode 
    opcode = SET_GPIO
    payload = [motors_pins[motor_num][0], 1]
    calculated_checksum = calculate_checksum(payload)
    packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
    bus.write_i2c_block_data(arduino_address, 0x01, packet)

    opcode = SET_PWM
    payload = [motors_pins[motor_num][1], 50]
    calculated_checksum = calculate_checksum(payload)
    packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
    bus.write_i2c_block_data(arduino_address, 0x01, packet)

    if(motor_num != 3):
        opcode = SET_PWM
        payload = [motors_pins[motor_num][2], 0]
        calculated_checksum = calculate_checksum(payload)
        packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
        bus.write_i2c_block_data(arduino_address, 0x01, packet)
    else:
        opcode = SET_GPIO
        payload = [motors_pins[motor_num][2], 1]
        calculated_checksum = calculate_checksum(payload)
        packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
        bus.write_i2c_block_data(arduino_address, 0x01, packet)
    print("started motor", motor_num)

def startMotorReverseDirection(motor_num):
    global opcode 
    opcode = SET_GPIO
    payload = [motors_pins[motor_num][0], 1]
    calculated_checksum = calculate_checksum(payload)
    packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
    bus.write_i2c_block_data(arduino_address, 0x01, packet)


    if(motor_num != 3):
        opcode = SET_PWM
        payload = [motors_pins[motor_num][1], 0]
        calculated_checksum = calculate_checksum(payload)
        packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
        bus.write_i2c_block_data(arduino_address, 0x01, packet)

        opcode = SET_PWM
        payload = [motors_pins[motor_num][2], 50]
        calculated_checksum = calculate_checksum(payload)
        packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
        bus.write_i2c_block_data(arduino_address, 0x01, packet)
    else:
        opcode = SET_PWM
        payload = [motors_pins[motor_num][1], 50]
        calculated_checksum = calculate_checksum(payload)
        packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
        bus.write_i2c_block_data(arduino_address, 0x01, packet)

        opcode = SET_GPIO
        payload = [motors_pins[motor_num][2], 0]
        calculated_checksum = calculate_checksum(payload)
        packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
        bus.write_i2c_block_data(arduino_address, 0x01, packet)
    print("started motor", motor_num)


def stopMotor(motor_num):
    global opcode 
    opcode = SET_GPIO
    payload = [motors_pins[motor_num][0], 0]
    calculated_checksum = calculate_checksum(payload)
    packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
    bus.write_i2c_block_data(arduino_address, 0x01, packet)

    opcode = SET_PWM
    payload = [motors_pins[motor_num][1], 0]
    calculated_checksum = calculate_checksum(payload)
    packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
    #print(packet)
    bus.write_i2c_block_data(arduino_address, 0x01, packet)

    if(motor_num != 3):
        opcode = SET_PWM
        payload = [motors_pins[motor_num][2], 0]
        calculated_checksum = calculate_checksum(payload)
        packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
        bus.write_i2c_block_data(arduino_address, 0x01, packet)
    else:
        opcode = SET_GPIO
        payload = [motors_pins[motor_num][2], 0]
        calculated_checksum = calculate_checksum(payload)
        packet = [start_byte, opcode, 2] + payload + [calculated_checksum]
        bus.write_i2c_block_data(arduino_address, 0x01, packet)
    print("stopped motor", motor_num)

try:
    
    startMotor(3)
    startMotor(2)
    startMotor(1)

    time.sleep(5)

    startMotorReverseDirection(3)
    startMotorReverseDirection(2)
    startMotorReverseDirection(1)
    
    time.sleep(5)
    
    stopMotor(3)
    stopMotor(2)
    stopMotor(1)
    
        
        #########################################################
        
        

except KeyboardInterrupt:
    print("Stopped sending packets.")
