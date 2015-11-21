#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import smbus
import math


# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
set_scale = 0x1b
sample_rate = 0x19

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command


offset_z = 0.0

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


def get_gyro_z():
	return (read_word_2c(0x47) - offset_z)


def save_offset():
	i = 0
	n = 500.0
	sum = 0.0
	while(i<n):
		sum = sum + read_word_2c(0x47)
		i = i + 1
	sum = sum / n
	print ("offset_z: ", sum)
	return sum

def init():
	global offset_z
	# Wake the 6050 up as it starts in sleep mode
	bus.write_byte_data(address, power_mgmt_1, 0)
	bus.write_byte_data(address, set_scale, 2)   # 2->+-1000grad/sec
	bus.write_byte_data(address, sample_rate, 7)   # 7->1kHz min value, already quite enought
	offset_z = save_offset()


# Test of the module
if __name__ == "__main__":
	init()
	j = 0
	sum = 0.0
	while(j<10000):
		val = get_gyro_z()
		
		print ("z centre :" , val)
		j = j + 1
		sum += val
	sum /= 10000.0
	print ("moyenne : ", sum)
