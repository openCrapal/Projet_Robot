#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import time
import mpu6050
mpu6050.init()
import localisation as loc
import pwmMotors
import automation as Z
import sys
import signal

kva = 25.
kpa = 0.01 #  0.005
tpa = .2   #  4.0
kda = 0.00001 # 0.00001
sata= 100.0

def fermer_pgrm(signal, frame):
	print("fermer proprement")
	import RPi.GPIO as GPIO
	GPIO.cleanup()
	sys.exit(0)

signal.signal(signal.SIGINT, fermer_pgrm)

Gyro = Z.Z_Filter(Z.Z_Sensor(mpu6050.get_gyro_z),0.8)
Goal = Z.Z_Constant(10.0)
D_Gyro = Z.Z_Filter(Z.Z_Derivative(Gyro), 0.8)
D_Goal = Z.Z_Derivative(Goal)

Orientation = Z.Z_Filter(Z.Z_Sensor(loc.get_teta), 0.4)

Motor =  Z.Z_PID(kva, kpa, tpa, kda, sata, Goal, Gyro, D_Goal, D_Gyro)

for i in range(1, 10000, 1):
	loc.update()
	#print(Gyro.get_val(), "\t", Goal.get_val(), "\t", Motor.get_val())
	m = Motor.get_val()
	if m >0:
		m += 3
	else:
		m -= 3
	pwmMotors.set_speed(m,m)
	#time.sleep(0.01)
	Z.Z_Index += 1

import RPi.GPIO as GPIO
GPIO.cleanup()

