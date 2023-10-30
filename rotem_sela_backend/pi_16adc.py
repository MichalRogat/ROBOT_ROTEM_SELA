#!/usr/bin/python
#===========================================
# Modified by Gil Medel June 2019 Modified to test board operation
# Setup to test Ch1 on address 0x76 (HIGH, HIGH, HIGH jumper setting) 
# I2C MUST BE ENABLED ON THE PI
#===========================================
import time
import smbus
from smbus import SMBus
from sys import exit
import pwm_rpi
# for Pi3B+ use SMBus(1)
bus = SMBus(1)
#Default    ADDRESS     A2    A1     A0
address =  0b1110110 #  HIGH  HIGH   HIGH    0x76
channel0    = 0xB0
channel1    = 0xB8
channel2    = 0xB1
channel3    = 0xB9
channel4    = 0xB2
channel5    = 0xBA
channel6    = 0xB3
channel7    = 0xBB
channel8    = 0xB4
channel9    = 0xBC
channel10   = 0xB5
channel11   = 0xBD
channel12   = 0xB6
channel13   = 0xBE
channel14   = 0xB7
channel15   = 0xBF
vref = 5.0
max_reading = 8388608.0


#===========================================
# time.sleep(I2C_sleep_time)
# ch0_mult = 1 # This is the multiplier value to read the Current used by the Pi.
# ch1_mult = 1 # Multiplier for Channel 1
# # simple waterfall code as below.
# while (True):
#     Ch0Value = ch0_mult*getreading(address, channel0)
#     print("Channel 0 at %s is %12.2f" % (time.ctime(), Ch0Value))
#     # Sleep between each reading.
#     time.sleep(I2C_sleep_time)
#     sys.stdout.flush()
#     time.sleep(sleep_time)
# # End of main loop.
#===========================================
byte_block_size = 0x06 # number of bytes to read in the block
sleep_time = .001 # number of seconds to sleep between each measurement
I2C_sleep_time = 0.2 # seconds to sleep between each channel reading
# I2C_sleep_time - has to be more than 0.2 (seconds).
#===========================================
def GetAdcPerMotor(motor):
    if motor == pwm_rpi.RobotMotor.Drive1.value:
        adc_channel=channel0
    elif motor==pwm_rpi.RobotMotor.Drive2.value:
        adc_channel=channel1
    elif motor==pwm_rpi.RobotMotor.Elev1.value:
        adc_channel=channel2        
    elif motor==pwm_rpi.RobotMotor.Elev2.value:
        adc_channel=channel3
    elif motor==pwm_rpi.RobotMotor.Turn1.value:
        adc_channel=channel4
    elif motor==pwm_rpi.RobotMotor.Turn2.value:
        adc_channel=channel5
    elif motor==pwm_rpi.RobotMotor.Joint1.value:
        adc_channel=channel6
    elif motor==pwm_rpi.RobotMotor.Joint2.value:
        adc_channel=channel7
    elif motor==pwm_rpi.RobotMotor.Roller.value:
        adc_channel=channel8
    elif motor==pwm_rpi.RobotMotor.Pump1.value:
        adc_channel=channel9
    elif motor==pwm_rpi.RobotMotor.Pump2.value:
        adc_channel=channel10        
    elif motor==pwm_rpi.RobotMotor.Pump3.value:
        adc_channel=channel11
    elif motor==pwm_rpi.RobotMotor.Pump4.value:
        adc_channel=channel12
    # elif motor==pwm_rpi.RobotMotor.Drive1.value:
    #     adc_channel=channel13
    # elif motor==pwm_rpi.RobotMotor.Drive1.value:
    #     adc_channel=channel14
    # elif motor==pwm_rpi.RobotMotor.Drive1.value:
    #     adc_channel=channel15
    return adc_channel

def GetA2d(motor):
    adc_channel=GetAdcPerMotor(motor)
    bus.write_byte(address, adc_channel)
    time.sleep(I2C_sleep_time)
    reading  = bus.read_i2c_block_data(address, adc_channel, byte_block_size)
    time.sleep(I2C_sleep_time)
#  Start conversion for the Channel Data
    valor = ((((reading[0]&0x3F))<<16))+((reading[1]<<8))+(((reading[2]&0xE0)))
#    print("Valor is 0x%x" % valor)
    volts = valor*vref/max_reading
    return volts