#!/usr/bin/python

import smbus
import math
import time

# default adress of the grove motor controller
motor_adress = 0x0f

# registers
set_freq = 0x84
motor_1 = 0xa1
motor_2 = 0xa5

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val


def save_offset():
	i = 0
	n = 100.0
	sum = 0.0
	while(i<n):
		sum = sum + read_word_2c(0x47)
		i = i + 1
	sum = sum / n
	print "offset_z: ", sum
	return sum

def init():
	# to begin, set the pwm frequency
	try:
		bus.write_byte_data(motor_adress, set_freq, 2)
	except IOError:
		# subprocess.call(['i2cdetect', '-y', '1'])
		# flag = 1     #optional flag to signal your code to resend or something
		pass

#Test of the module
if __name__ == "__main__":
	init()
	
