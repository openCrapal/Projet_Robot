#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import i2cBus.grove_motor as i2c
import i2cBus.mpu6050 as i2c

i2c.Grove_motor.write_byte(0xa1, -50)

#set_freq = 0x84
# motor_1 = 0xa1
# motor_2 = 0xa5


