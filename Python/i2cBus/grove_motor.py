#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import smbus
import math
import time
from i2cFunctions import *

# default adress of the grove motor controller
motor_adress = 0x0f

# registers
set_freq = 0x84
motor_1 = 0xa1
motor_2 = 0xa5

Grove_motor = I2cDevice(motor_adress)

# to begin, set the pwm frequency
try:
	Grove_motor.write_byte(set_freq, 4)
except IOError:
	print(" Error connecting to motor at the adress: ", motor_adress)
	#subprocess.call(['i2cdetect', '-y', '1'])
	#flag = 1     #optional flag to signal your code to resend or something
	#pass

#Test of the module
if __name__ == "__main__":
	Grove_motor.write_byte(0xa1, -50)
	
	
