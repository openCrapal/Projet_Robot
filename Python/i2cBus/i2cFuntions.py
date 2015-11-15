#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import smbus

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards

def read_byte(adr):
	return bus.read_byte_data(motor_adress, adr)

def read_word(adr):
	high = bus.read_byte_data(motor_adress, adr)
	low = bus.read_byte_data(motor_adress, adr+1)
	val = (high << 8) + low
	return val

def read_word_2c(adr):
	val = read_word(adr)
	if (val >= 0x8000):
		return -((65535 - val) + 1)
	else:
		return val

class I2cDevice:
	# define any i2c device with his adress
	# propose somme usefull methods
	def __init__(self, adress):
		self.device_adress = adress

	def read_byte(adr):
		try:
			return bus.read_byte_data(self.device_adress, adr)
		except IOError:
			print(" Error connecting to device at the adress: ", self.device_adress)
			subprocess.call(['i2cdetect', '-y', '1'])
			flag = 1     #optional flag to signal your code to resend or something
			pass

	def read_word(adr):
		high = bus.read_byte_data(self.device_adress, adr)
		low = bus.read_byte_data(self.device_adress, adr+1)
		val = (high << 8) + low
		return val

	def read_word_2c(adr):
		val = read_word(adr)
		if (val >= 0x8000):
			return -((65535 - val) + 1)
		else:
			return val
