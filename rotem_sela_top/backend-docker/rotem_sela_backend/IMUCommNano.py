import smbus2
import time
import math
import _thread
import threading
from functions_2 import GenericFunctions, ArduinoAddress


class MinIMU_v5_pi:
    """
    Init function
    Valid values for    aFullScale are 2, 4, 8, and 16 [g]
                                            gFullScale are 125, 245, 500, 1000, and 2000 [dps]
                                            mFullScale are 4, 8, 12, and 16 [guass]
    """

    def __init__(self, address, lock, SMBusNum=1, aFullScale=2, gFullScale=500, mFullScale=4):
        if address == 0:
            # i2c addresses
            self.mag = 0x1e  # 0011110 (from docs)
            self.accel_gyro = 0x6b
        else:
            # i2c addresses
            self.mag = 0x1c  # 0011100 (from docs)
            self.accel_gyro = 0x6a
        self.lock = lock
        # Accelerometer and Gyro Register addresses
        self.Accel_Gyro_REG = dict(
            FUNC_CFG_ACCESS=0x01,

            FIFO_CTRL1=0x06,
            FIFO_CTRL2=0x07,
            FIFO_CTRL3=0x08,
            FIFO_CTRL4=0x09,
            FIFO_CTRL5=0x0A,
            ORIENT_CFG_G=0x0B,
                                                        \
            INT1_CTRL=0x0D,
            INT2_CTRL=0x0E,
            WHO_AM_I=0x0F,
            CTRL1_XL=0x10,
            CTRL2_G=0x11,
            CTRL3_C=0x12,
            CTRL4_C=0x13,
            CTRL5_C=0x14,
            CTRL6_C=0x15,
            CTRL7_G=0x16,
            CTRL8_XL=0x17,
            CTRL9_XL=0x18,
            CTRL10_C=0x19,
                                                        \
            WAKE_UP_SRC=0x1B,
            TAP_SRC=0x1C,
            D6D_SRC=0x1D,
            STATUS_REG=0x1E,
                                                        \
            OUT_TEMP_L=0x20,
            OUT_TEMP_H=0x21,
            OUTX_L_G=0x22,
            OUTX_H_G=0x23,
            OUTY_L_G=0x24,
            OUTY_H_G=0x25,
            OUTZ_L_G=0x26,
            OUTZ_H_G=0x27,
            OUTX_L_XL=0x28,
            OUTX_H_XL=0x29,
            OUTY_L_XL=0x2A,
            OUTY_H_XL=0x2B,
            OUTZ_L_XL=0x2C,
            OUTZ_H_XL=0x2D,
                                                        \
            FIFO_STATUS1=0x3A,
            FIFO_STATUS2=0x3B,
            FIFO_STATUS3=0x3C,
            FIFO_STATUS4=0x3D,
            FIFO_DATA_OUT_L=0x3E,
            FIFO_DATA_OUT_H=0x3F,
            TIMESTAMP0_REG=0x40,
            TIMESTAMP1_REG=0x41,
            TIMESTAMP2_REG=0x42,
                                                        \
            STEP_TIMESTAMP_L=0x49,
            STEP_TIMESTAMP_H=0x4A,
            STEP_COUNTER_L=0x4B,
            STEP_COUNTER_H=0x4C,
                                                        \
            FUNC_SRC=0x53,
                                                        \
            TAP_CFG=0x58,
            TAP_THS_6D=0x59,
            INT_DUR2=0x5A,
            WAKE_UP_THS=0x5B,
            WAKE_UP_DUR=0x5C,
            FREE_FALL=0x5D,
            MD1_CFG=0x5E,
            MD2_CFG=0x5F)

        # Magnemometer addresses
        self.Mag_REG = dict(
            WHO_AM_I=0x0F,
                                        \
            CTRL_REG1=0x20,
            CTRL_REG2=0x21,
            CTRL_REG3=0x22,
            CTRL_REG4=0x23,
            CTRL_REG5=0x24,
                                        \
            STATUS_REG=0x27,
            OUT_X_L=0x28,
            OUT_X_H=0x29,
            OUT_Y_L=0x2A,
            OUT_Y_H=0x2B,
            OUT_Z_L=0x2C,
            OUT_Z_H=0x2D,
            TEMP_OUT_L=0x2E,
            TEMP_OUT_H=0x2F,
            INT_CFG=0x30,
            INT_SRC=0x31,
            INT_THS_L=0x32,
            INT_THS_H=0x33)

        # Unit scales
        self.aScale = 0  # default: aScale = 2g/2^15,
        self.gScale = 0  # default: gScale = 500dps/2^15
        self.mScale = 0  # default: mScale = 4guass/2^15

        # Variables for updateAngle and updateYaw
        self.prevAngle = [[0, 0, 0]]  # x, y, z (roll, pitch, yaw)
        self.prevYaw = [0]
        self.tau = 0.04  # Want this roughly 10x the dt
        self.lastTimeAngle = [0]
        self.lastTimeYaw = [0]

        # Connect i2c bus
        self.bus = smbus2.SMBus(SMBusNum)

    """Combines Hi and Low 8-bit values to a 16-bit two's complement and
	converts to decimal"""

    def byteToNumber(self, val_Low, val_Hi):
        number = 256 * val_Hi + val_Low  # 2^8 = 256
        if number >= 32768:  # 2^7 = 32768
            number = number - 65536  # For two's complement
        return number


def main():
    i2c_lock = threading.Lock()
    bus = smbus2.SMBus(1)
    IMU = MinIMU_v5_pi(0, i2c_lock)
    while True:
        for address in ArduinoAddress:  # Arduino1, Arduino2, Arduino3, Arduino4, Arduino5
            # 0x11, 0x12, 0x13, 0x14, 0x15
            GenericFunctions.readIMU(hex(address.value))
            data = bus.read_i2c_block_data(hex(address.value), 0, 12)
            print(data)
            time.sleep(0.1)
    """while True:
				i = 0
				while i < 30:
					i += 1
					IMU.updateYaw()
					time.sleep(0.004)           
				print IMU.prevYaw[0]
				#print  IMU.readAccelerometer() + IMU.readGyro() + IMU.readMagnetometer()
				time.sleep(0.004)"""


if __name__ == "__main__":
    main()
