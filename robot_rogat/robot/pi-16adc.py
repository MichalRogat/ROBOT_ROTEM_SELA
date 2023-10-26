#!/usr/bin/python
#===========================================
# Modified by Gil Medel June 2019 Modified to test board operation
# Setup to test Ch1 on address 0x76 (HIGH, HIGH, HIGH jumper setting) 
# I2C MUST BE ENABLED ON THE PI
#===========================================
import time
import smbus
import sys
import os
import subprocess

from smbus import SMBus
from sys import exit
# for Pi3B+ use SMBus(1)
bus = SMBus(1)
#Default    ADDRESS     A2    A1     A0
address =  0b1110110 #  HIGH  HIGH   HIGH    0x76
channel0 = 0xB0
vref = 2.5
max_reading = 8388608.0
#===========================================
byte_block_size = 0x06 # number of bytes to read in the block
sleep_time = .001 # number of seconds to sleep between each measurement
I2C_sleep_time = 0.2 # seconds to sleep between each channel reading
# I2C_sleep_time - has to be more than 0.2 (seconds).
#===========================================
def getreading(adc_address,adc_channel):
    bus.write_byte(adc_address, adc_channel)
    time.sleep(I2C_sleep_time)
    reading  = bus.read_i2c_block_data(adc_address, adc_channel, byte_block_size)
#  Start conversion for the Channel Data
    valor = ((((reading[0]&0x3F))<<16))+((reading[1]<<8))+(((reading[2]&0xE0)))
#    print("Valor is 0x%x" % valor)
    volts = valor*vref/max_reading
    return volts
#===========================================
time.sleep(I2C_sleep_time)
ch0_mult = 1 # This is the multiplier value to read the Current used by the Pi.
ch1_mult = 1 # Multiplier for Channel 1
# simple waterfall code as below.
while (True):
    Ch0Value = ch0_mult*getreading(address, channel0)
    print("Channel 0 at %s is %12.2f" % (time.ctime(), Ch0Value))
    # Sleep between each reading.
    time.sleep(I2C_sleep_time)
    sys.stdout.flush()
    time.sleep(sleep_time)
# End of main loop.