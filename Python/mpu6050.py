#!/usr/bin/python

import smbus
import math


# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

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
	n = 100.0
	sum = 0.0
	while(i<n):
		sum = sum + read_word_2c(0x47)
		i = i + 1
	sum = sum / n
	print "offset_z: ", sum
	return sum

def init():
	# Wake the 6050 up as it starts in sleep mode
	bus.write_byte_data(address, power_mgmt_1, 0)
	offset_z = save_offset()

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Test of the module
if __name__ == "__main__":
	init()
	print "gyro data"
	print "---------"

	gyro_xout = read_word_2c(0x43)
	gyro_yout = read_word_2c(0x45)
	#gyro_zout = read_word_2c(0x47)
	gyro_zout = get_gyro_z()

	print "gyro_xout: ", gyro_xout, " scaled: ", (gyro_xout / 131)
	print "gyro_yout: ", gyro_yout, " scaled: ", (gyro_yout / 131)
	print "gyro_zout: ", gyro_zout, " scaled: ", (gyro_zout / 131)

	print
	print "accelerometer data"
	print "------------------"

	accel_xout = read_word_2c(0x3b)
	accel_yout = read_word_2c(0x3d)
	accel_zout = read_word_2c(0x3f)

	accel_xout_scaled = accel_xout / 16384.0
	accel_yout_scaled = accel_yout / 16384.0
	accel_zout_scaled = accel_zout / 16384.0

	print "accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled
	print "accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled
	print "accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled

	j = 0
	sum = 0.0
	while(j<10):
		val = get_gyro_z()
		
		print "z centre :" , val
		j = j + 1
		sum += val
	sum /= 10.0
	print "moyenne : ", sum
