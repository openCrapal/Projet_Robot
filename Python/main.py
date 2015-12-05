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

edit_file = False
if edit_file:
	import os
	os.chdir("/home/pi/Documents/results")
	print (os.getcwd())
	my_file = open(time.strftime("%B_%d_%H_%M_%S"), 'w')
	my_file.write("time(s)\tmpu6050(rad/s/s\tmotors(% @ {0}V) \n".format(pwmMotors.pwmAmpli))

# I use a 12 V alim for the motors. I set Z_Ampli to 12
# When I swich to 6V, just have to set Z_Ampli to 6
# Well, if your system is linear, and the saturator clairly isn't
Ampli = 12.0
# position of the weightPoint
radius_G = 0.2

# PI inclinaison, the value proposed are are the robust ones (half the limit ones) by 12V
kva = 1.0     #1.0
kpa = 45.     #55
tpa = 0.0002  #0.0002
kda = 1.0     #1.0
sata= 80.0    #100
t_filter_a = 0.15

# PID orientation
kvo = 1.2 #1.2
kpo = 0.0 # 400
tpo = 0.0006 #  0.0006
kdo = 0.0# 0.001
sato= 50
t_filter_o = 0.1

# PID position
kvw = 0.1
kpw = 1.0
tpw = 0.0004
kdw = 0.0
satw= 0.0
t_filter_w = 2.0

# loop time, seconds. Must be more than the actual time it takes to free CPU use for other process
loop_time = 0.010
t_begin_program = time.time()

def fermer_pgrm(signal, frame):
	print("fermer proprement")
	import RPi.GPIO as GPIO
	GPIO.cleanup()
	if edit_file:
		my_file.close()
	sys.exit(0)
signal.signal(signal.SIGINT, fermer_pgrm)

# The trajectorie must be generated, time continuus. Constant zero is as continuus as it get's
W_Goal = Z.Z_Constant(0.0)
V_Goal = Z.Z_Derivative(W_Goal)

# Rotation speed of the bot afak falling speed
Gyro = Z.Z_Filter(Z.Z_Gain(Z.Z_Sensor(mpu6050.get_gyro_y),-0.00213), t_filter_a)
D_Gyro = Z.Z_Constant(0.0)
# Speed of the point between the wheels M
V_M = Z.Z_Sensor(loc.get_speed)
# Speed of the weightPoint G
V_G = Z.Z_Filter(Z.Z_Sum(Gyro, V_M, radius_G, 1.0), t_filter_w)
# Absolute position of the weightPoint (G) on it's trajectorie
Way_G = Z.Z_Filter(Z.Z_Sum(Z.Z_Sensor(loc.get_way), Z.Z_Integral(Gyro, 2.0), 1.0, radius_G), t_filter_w)

# Falling Speed you want to achieve, I'd say not too fast!
I_Goal = Z.Z_PID(kvw, kpw, tpw, kdw, satw, W_Goal, Way_G, V_Goal, V_G)
D_I_Goal= Z.Z_Constant(0.0)
#I_Goal = Z.Z_Constant(3.0)

# All about orientation
Orientation = Z.Z_Filter(Z.Z_Sensor(loc.get_teta), 0.1)
D_Orientation = Z.Z_Filter(Z.Z_Derivative(Orientation), 0.5)
Teta_Goal = Z.Z_Constant(0.0)
D_Teta_Goal = Z.Z_Derivative(Teta_Goal)

Dir   = Z.Z_Gain(Z.Z_PID(kvo, kpo, tpo, kdo, sato*Ampli, Teta_Goal, Orientation, D_Teta_Goal, D_Orientation), 1/Ampli)
Motor = Z.Z_Gain(Z.Z_PID(kva, kpa, tpa, kda, sata*Ampli, I_Goal, Gyro, D_I_Goal, D_Gyro), 1/Ampli)

while (time.time()-t_begin_program  < 10):
	t_begin_loop = time.time()
	loc.update()
	W_Goal.set_val((time.time() - t_begin_program)*0.01000)
	#Teta_Goal.set_val( (time.time()-t2)/5.0)
	#print(Gyro.get_val())
	d = Dir.get_val()
	m = Motor.get_val()
	pwmMotors.set_speed(m-d,m+d)
	#print("w: {0}\tv: {1}\tI_soll: {2}\tWgoal: {3} ".format(Way_G.get_val(), V_G.get_val(), I_Goal.get_val(), W_Goal.get_val()))
	Z.Z_Index += 1

	if edit_file:
		my_file.write("{0}\t{1}\t{2}\n".format(time.time()-t_begin_program, Gyro.get_val(), m/pwmMotors.pwmAmpli))

	t2 = loop_time + t_begin_loop - time.time()
	#print (t2)
	if t2<0:
		t2=0
	time.sleep(t2)

import RPi.GPIO as GPIO
GPIO.cleanup()
print("erreurs de localisations : ", loc.pos_errors)
if edit_file:
	my_file.close()

