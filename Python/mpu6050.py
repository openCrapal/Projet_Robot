#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import smbus
import math
from time import sleep


# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
set_scale = 0x1b
sample_rate = 0x19
low_pass_filter = 0x1a
bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

scale = 1.0
offset_x = offset_y = offset_z = 0

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

z_old = 0.0
def get_gyro_z():
	global z_old
	try:
		v = read_word_2c(0x47)
	except:
		v = z_old
	else:
		z_old = v
	finally:
		return (v - offset_z) * scale

y_old = 0.0
def get_gyro_y():
	global y_old
	try:
		v = read_word_2c(0x45)
	except:
		v = y_old
	else:
		y_old = v
	finally:
		return (v - offset_y) * scale

x_old = 0.0
def get_gyro_x():
	global x_old
	try:
		v = read_word_2c(0x43)
	except:
		v = x_old
	else:
		x_old = v
	finally:
		return (v - offset_x) * scale



def save_offset(n=100):
	global offset_x, offset_y, offset_z
	offset_x = offset_y = offset_z =0
	for i in range(1, n, 1):
		offset_x += read_word_2c(0x43)
		offset_y += read_word_2c(0x45)
		offset_z += read_word_2c(0x47)
	offset_x/=n
	offset_y/=n
	offset_z/=n
	print ("offset_x: ", offset_x, "\t offset_y: ", offset_y, "\t offset_z :", offset_z)

def init():
	global scale
	# Wake the 6050 up as it starts in sleep mode
	try:
		bus.write_byte_data(address, power_mgmt_1, 0)
	except:
		print("Connexion mpu6050 failed, trying again...")
		sleep(0.01)
		bus.write_byte_data(address, power_mgmt_1, 0)
		pass
	bus.write_byte_data(address, set_scale, 2)   # 2->+-1000grad/sec
	scale = 1000 * math.pi / (180 * 32767)
	bus.write_byte_data(address, sample_rate, 7)   # 7->1kHz min value, already quite enought
	bus.write_byte_data(address, low_pass_filter, 0x4) # 6->5Hz ; 0->256Hz
	save_offset()


# Test of the module
if __name__ == "__main__":
	init()
	for j in range(1, 200, 1):
		print ("x: ", get_gyro_x(), "\t y: ", get_gyro_y(), "\t z:" , get_gyro_z())

