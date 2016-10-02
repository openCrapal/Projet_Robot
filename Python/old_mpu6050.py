#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import smbus
import math
from time import sleep, time


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



def save_offset(n=500):
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
	bus.write_byte_data(address, set_scale, 2)		# 2->+-1000degrees/sec
	scale = 1000 * 3.141 / (8 * 32768 * 180)		# Rad / sec
	#scale = 1
	bus.write_byte_data(address, sample_rate, 0)		# 7->1kHz min value, already quite enought
	bus.write_byte_data(address, low_pass_filter, 0x0)	# 6->5Hz ; 0->256Hz
	save_offset()


# Test of the module
if __name__ == "__main__":
	init()
	sleep(0.5)
	t = t_old = timer = time()
	t_next = timer + 0.1
	Ix = Iy = Iz = 0.0
	n = 0
	while(t - timer < 10.0):
		dt = t - t_old
		x = get_gyro_x()
		y = get_gyro_y()
		z = get_gyro_z()
		Ix += dt * x
		Iy += dt * y
		Iz += dt * z
		t_old = t
		t = time()
		if( t>= t_next):
			print (Ix, "\t;", Iy, "\t;" , Iz, "\t;", n)
			t_next += 0.05
			n = 0
		else:
			n += 1
		#sleep(0.001)
