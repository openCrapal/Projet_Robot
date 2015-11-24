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

# I use a 12 V alim for the motors. I set Z_Ampli to 12
# When I swich to 6V, just have to set Z_Ampli to 6
# Well, if your system is linear, and the saturator clairly isn't
pwmMotors.pwmAmpli = 12.0

# PID inclinaison, the value proposed are are the robust ones (half the limit ones) by 12V
kva = 0.1  #  0.1
kpa = 0.4  #  0.4
tpa = 0.3   #  0.3
kda = 0.005 # 0.005
sata= 80.0   # 100
t_filter_a = 0.8

# PID orientation
kvo = 1.2
kpo = 400 # 400
tpo = 0.0006 #  0.0006
kdo = 0.001# 0.001
sato= 30.0
t_filter_o = 0.1

# loop time, seconds. Must be more than the actual time it takes to free CPU use for other process
loop_time = 0.04
t_begin_program = time.time()

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

Orientation = Z.Z_Filter(Z.Z_Sensor(loc.get_teta), 0.1)
D_Orientation = Z.Z_Filter(Z.Z_Derivative(Orientation), 0.1)
Teta_Goal = Z.Z_Constant(0.0)
D_Teta_Goal = Z.Z_Derivative(Teta_Goal)

Dir   = Z.Z_PID(kvo, kpo, tpo, kdo, sato, Teta_Goal, Orientation, D_Teta_Goal, D_Orientation)
Motor = Z.Z_PID(kva, kpa, tpa, kda, sata, Goal, Gyro, D_Goal, D_Gyro)

time.sleep(0.2)
t2=time.time()
while (time.time() - t_begin_program <  10):
	t_begin_loop = time.time()
	loc.update()
	Teta_Goal.set_val( (time.time()-t2)/5.0)
	#print(Gyro.get_val(), "\t", Goal.get_val(), "\t", Motor.get_val())
	d = Dir.get_val()
	m = Motor.get_val()
	pwmMotors.set_speed(m-d,m+d)
	#time.sleep(0.01)
	Z.Z_Index += 1
	time.sleep(loop_time + t_begin_loop - time.time())

import RPi.GPIO as GPIO
GPIO.cleanup()
print("erreurs de localisations : ", loc.pos_errors)
