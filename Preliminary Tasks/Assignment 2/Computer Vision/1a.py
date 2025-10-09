'''
This program demonstrates communication between the Pi and Arduino.
'''

from smbus2 import SMBus
from time import sleep
# I2C address of the Arduino, set in Arduino sketch
ARD_ADDR = 8
# Initialize SMBus library with I2C bus 1
i2c = SMBus(1)

offset = 0

# Do in a loop
while(True):
    # Get user input for command
    string = input("Enter a string of 32 characters or less:")
    # Write a byte to the i2c bus
    command = [ord(character) for character in string]
    i2c.write_i2c_block_data(ARD_ADDR,offset,command)
