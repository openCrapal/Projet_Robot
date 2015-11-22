#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import time
import mpu6050
mpu6050.init()
import localisation
import pwmMotors
import automation as Z
import sys
import signal

kva = 01.0
kpa = 1000
tpa = 100000.0
kda = 0.0
sata= 20.0

def fermer_pgrm(signal, frame):
	print("fermer proprement")
	import RPi.GPIO as GPIO
	GPIO.cleanup()
	sys.exit(0)

signal.signal(signal.SIGINT, fermer_pgrm)

Gyro = Z.Z_Filter(Z.Z_Sensor(mpu6050.get_gyro_z), 0.2)
Goal = Z.Z_Constant(10.0)

Motor =  Z.Z_PID(kva, kpa, tpa, kda, sata, Goal, Gyro)

for i in range(1, 100, 1):
	print(Gyro.get_val(), "\t", Goal.get_val(), "\t", Motor.get_val())
	
	pwmMotors.set_speed(Motor.get_val(), Motor.get_val())
	time.sleep(0.02)
	Z.Z_Index += 1

import RPi.GPIO as GPIO
GPIO.cleanup()

