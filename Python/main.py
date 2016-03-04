#!/usr/bin/python3.4
# -*-coding:utf-8 -*

from constants import *
import time
from i2cDev import i2c_devices
import localisation
import automation as Z
import sys
import signal
import math

edit_file = False
if edit_file:
	import os
	os.chdir("/home/pi/Documents/results")
	print (os.getcwd())
	my_file = open(time.strftime("%B_%d_%H_%M_%S"), 'w')
	my_file.write("time(s)\tmpu6050\tphase=5rad/sec")

# Heigh of the weightPoint relative to the wheels axis
radius_G = 0.1

# PI inclinaison, the value proposed are are the robust ones (half the limit ones) by 12V
kva = 20.0     #1.0
kpa = 8000.    #800
tpa = 0.0001    #0.0001
kda = 0.     #1.0
kaa = 40.0     # 40 à t_filter = 0.4
sata= 100.0    #100
t_filter_a = 0.5 # 0.4

# PID orientation
kvo = 0.5#1.2
kpo = 100. # 400
tpo = 0.0001 #  0.0006
kdo = 0.001# 0.001
sato= 0
t_filter_o = 0.05

# PID position
kvw = 1.
kpw = 0.0
tpw = 1000.4
kdw = 0.0
satw= 00.0
t_filter_w = 2.0

# loop time, seconds. Must be more than the actual time it takes to free CPU use for other process
loop_time = 0.005
t_begin_program = time.time()

def fermer_pgrm(signal, frame):
	print("fermer proprement")
	import RPi.GPIO as GPIO
	GPIO.cleanup()
		
	try:
		loc.finish()
	except:
		pass

	if edit_file:
		my_file.close()
	sys.exit(0)

signal.signal(signal.SIGINT, fermer_pgrm)

def go_robot_go(time_loop=8):
	Z.Z_Index = 0
	t_begin_program = time.time()
	loc = localisation.loc()
	loc.start()
	myI2cDevs = i2c_devices()
	myI2cDevs.start()
	# The trajectorie must be generated, time continuus. Constant zero is as continuus as it gets
	W_Goal = Z.Z_Constant(0.0)
	V_Goal = Z.Z_Derivative(W_Goal)

	# Rotation speed of the bot afak falling speed
	Gyro = Z.Z_Gain(Z.Z_Sensor(myI2cDevs.get_gyro_x), -1.0)
	D_Gyro = Z.Z_Derivative(Gyro)
	Estimated_Incl = Z.Z_Integral(Gyro, 6.0) # High-pass Filter to counter offset's integration

	# Speed of the point between the wheels: M
	V_M = Z.Z_Sensor(loc.get_speed)
	# Speed of the weightPoint: G
	V_G = Z.Z_Sum(Gyro, V_M, radius_G, 1.0)
	A_G = Z.Z_Derivative(V_G)
	# Absolute position of the weightPoint (G) on it's trajectorie
	Way_G = Z.Z_Sum(Z.Z_Sensor(loc.get_way), Estimated_Incl, 1.0, radius_G)

	# Inclinaison you want to achieve, depending on speed and position
	# This is the very critical point of a balancing bot without absolut level sensor
	#I_Goal = Z.Z_Filter(Z.Z_Sum(Z.Z_PID(kvw, kpw, tpw, kdw, satw, W_Goal, Way_G, V_Goal, V_G), A_G, 1.0, -kaw), t_filter_w)
	I_Goal = Z.Z_Constant(0.0)
	D_I_Goal= Z.Z_Derivative(I_Goal)

	# All about orientation
	Orientation = Z.Z_Filter(Z.Z_Sensor(loc.get_teta), 0.1)
	D_Orientation = Z.Z_Filter(Z.Z_Derivative(Orientation), 0.5)
	Teta_Goal = Z.Z_Constant(0.0)
	D_Teta_Goal = Z.Z_Derivative(Teta_Goal)

	Dir   = Z.Z_Filter(Z.Z_Gain(Z.Z_PID(kvo, kpo, tpo, kdo, sato*U_alim, Teta_Goal, Orientation, D_Teta_Goal, D_Orientation), 1/U_alim), t_filter_o)
	Motor = Z.Z_Filter(Z.Z_Sum(Z.Z_PID(kva, kpa, tpa, kda, sata*U_alim, I_Goal, Estimated_Incl, D_I_Goal, Gyro), D_Gyro, 1/U_alim, -kaa/U_alim), t_filter_a)
	#Motor = Z.Z_Filter(Z.Z_Sum(Z.Z_PID(kva, kpa, tpa, kda, sata*Ampli, I_Goal, Estimated_Incl, D_I_Goal, Gyro), D_Gyro, 1/Ampli, -kaa/U_alim), t_filter_a)
	
	max = 0.0
	min = 10.0
	while (time.time()-t_begin_program  < time_loop):
		t_begin_loop = time.time()
		#W_Goal.set_val(W_Goal.get_val() + 0.001)
		#Teta_Goal.set_val( Teta_Goal.get_val()+0.00)
#		print(I_Goal.get_val())
		#print(Gyro.get_val())
		#ampl = 0.1
		#phase = (time.time() - t_begin_program) * 6.281 * 1.0
		#I_Goal.set_val(ampl * math.sin(phase))
		d = Dir.get_val()
		m = Motor.get_val()
		myI2cDevs.set_speed( m+d, m-d)
#		pwmMotors.set_speed( ampl * math.sin(phase), ampl * math.sin(phase))
#		print("w: {0}\tv: {1}\tI_soll: {2}\tWgoal: {3} ".format(Way_G.get_val(), V_G.get_val(), I_Goal.get_val(), W_Goal.get_val()))
		Z.Z_Index += 1
		if edit_file:
			my_file.write("{0}\t{1}\t{2}\n".format(time.time()-t_begin_program, Gyro.get_val(), phase)) # m/pwmMotors.pwmAmpli))

		t2 = loop_time + t_begin_loop - time.time()
#		print (Estimated_Incl.get_val())
#		print (t2)
		if (t2>max) : max = t2
		if (t2<min) : min = t2
		if t2<0:
			t2=0
		time.sleep(t2)
	print("min: ", min, " ; max: ", max)
	loc.finish()
	myI2cDevs.set_speed(0,0)
	myI2cDevs.finish()
	# End go_robot_go

while True:
	mode = input("Quit: q ; Continue: c\n$ ")
	if (mode == "q" or mode == "Q"):
		import RPi.GPIO as GPIO
		GPIO.cleanup()
		if edit_file:
			my_file.close()
		exit()	
	else:
		go_robot_go(10)
		
import RPi.GPIO as GPIO
GPIO.cleanup()
if edit_file:
	my_file.close()

