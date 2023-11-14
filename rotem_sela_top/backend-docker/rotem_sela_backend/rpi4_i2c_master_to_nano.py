import smbus2
import time

# Define I2C address and other parameters
arduino_address = 0x55
start_byte = 0xAA
opcode = 0x01
payload = []
checksum = 0

# Initialize the I2C bus
bus = smbus2.SMBus(1)  # 1 indicates the I2C bus number

# Function to calculate the checksum
def calculate_checksum(packet):
    checksum = 0
    checksum += start_byte
    checksum += opcode
    checksum += (len(packet) & 0xFF)  # LSB of length
    checksum += (len(packet) >> 8)  # MSB of length
    for data in packet:
        checksum += data
    return (~checksum) & 0xFF

try:
    while True:
        # Calculate the checksum for the current payload
        calculated_checksum = calculate_checksum(payload)
        
        # Send the packet to the Arduino
        packet = [start_byte, opcode, 0, 0] + payload + [calculated_checksum]
        bus.write_i2c_block_data(arduino_address, 0x01, packet)
        
        # Print the sent packet
        print("Sent packet:", packet)
        
        # Delay for a while before sending the next packet
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopped sending packets.")
