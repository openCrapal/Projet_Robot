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

# PID inclinaison
kva = 0.0  #  25
kpa = 0.01 #  0.01
tpa = 1000000.1   #  0.2
kda = 0.0000 # 0.00001
sata= 50.0   # 100

# PID orientation
kvo = 0.5
kpo = 50 # 50
tpo = 0.0001   #  0.0001
kdo = 0.00# 0.00001
sato= 30.0

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

Orientation = Z.Z_Filter(Z.Z_Sensor(loc.get_teta), 0.8)
D_Orientation = Z.Z_Filter(Z.Z_Derivative(Orientation), 0.8)
Teta_Goal = Z.Z_Constant(0.0)
D_Teta_Goal = Z.Z_Filter(Z.Z_Derivative(Teta_Goal), 0.5)

Dir   = Z.Z_PID(kvo, kpo, tpo, kdo, sato, Teta_Goal, Orientation, D_Teta_Goal, D_Orientation)
Motor = Z.Z_PID(kva, kpa, tpa, kda, sata, Goal, Gyro, D_Goal, D_Gyro)

for i in range(1, 1000, 1):
	loc.update()
	Teta_Goal.set_val( 0.)
	#print(Gyro.get_val(), "\t", Goal.get_val(), "\t", Motor.get_val())
	d = Dir.get_val()
	m = Motor.get_val()
	pwmMotors.set_speed(m-d,m+d)
	#time.sleep(0.01)
	Z.Z_Index += 1

import RPi.GPIO as GPIO
GPIO.cleanup()
print("erreurs de localisations : ", loc.pos_errors)
