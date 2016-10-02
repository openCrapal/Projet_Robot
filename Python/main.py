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

myI2cDev = i2c_devices()
myI2cDev.save_gyro_offset(500)
myLoc = localisation.loc()
myI2cDev.start()
myLoc.start()

edit_file = False
if edit_file:
	import os
	os.chdir("/home/pi/Documents/results")
	print ("File open at ",os.getcwd())
	my_file = open(time.strftime("%B_%d_%H_%M_%S") + "txt", 'w' )
	my_file.write("time(s)\tmpu6050\testimated_incl\tU_mot\tpos_way")

time.sleep(0.2)

# PI inclinaison, the value proposed are are the robust ones (half the limit ones) by 12V
kva = 0.0      	#30.0
kpa = 4000.     	#1000
tpa = 1000.000004  	#0.00005
ksa = 100.		# % geschwindigkeitsvorsteuerung
kda = 0.0     		#10.0
kaa = .0		#5.0
sata= 150.0    		#200
#t_filter_a = 0.002 # 0.05

# PID orientation
kvo = 2. 	#1.2
kpo = 500. # 400
tpo = 0.000017 #  0.0006
kdo = 10.1# 0.001
sato= 0
t_filter_o = 0.001

# PID position
kvw = 0.2	# 0.5
kpw = 0.2
tpw = 1.6
ksw = 100.0		# %
kdw = 0.001
satw= .0
t_filter_w = 0.1

# loop time, seconds. Must be more than the actual time it takes to free CPU use for other process
loop_time = 0.002
t_begin_program = time.time()

def fermer_pgrm(signal, frame):
	print("fermer proprement")
	import RPi.GPIO as GPIO
	GPIO.cleanup()
		
	myLoc.finish()
	myI2cDev.finish()
	#del myLoc
	#del myI2cDev

	if edit_file:
		my_file.close()
	time.sleep(0.1)
	sys.exit(0)

signal.signal(signal.SIGINT, fermer_pgrm)

def go_robot_go(i2cdevice, localisation, time_prgm=8):
	Z.Z_Index = 0
	t_begin_program = time.time()
	localisation.reset()
	i2cdevice.reset()
	# The trajectorie must be generated, time continuus. Constant zero is as continuus as it gets
	W_Goal = Z.Z_Constant(0.0)
	V_Goal = Z.Z_Derivative(W_Goal)

	# Rotation speed of the bot afak falling speed
	Gyro = Z.Z_Sensor(i2cdevice.get_gyro_x)
	D_Gyro = Z.Z_Derivative(Gyro)
	Estimated_Incl = Z.Z_Sensor(i2cdevice.get_estimated_incl) 

	# Speed of the point between the wheels: M
	V_M = Z.Z_Sensor(localisation.get_speed)
	# Speed of the weightPoint: G
	V_G = Z.Z_Sum(Gyro, V_M, radius_G, 1.0)
	A_G = Z.Z_Derivative(V_G)
	# Absolute position of the weightPoint (G) on it's trajectorie
	Way_G = Z.Z_Sum(Z.Z_Sensor(localisation.get_way), Estimated_Incl, 1.0, radius_G)
	Ecart_V = Z.Z_Sum(V_Goal, V_G, ksw / 100.0, -ksw / 100.0)

	# Inclinaison you want to achieve, depending on speed and position
	# This is the very critical point of a balancing bot without absolut level sensor
	I_Goal = Z.Z_Filter(Z.Z_Gain(Z.Z_PID(kvw, kpw, tpw, kdw, satw, W_Goal, Way_G, Ecart_V, Z.Z_Constant(0.0) ), -1.0), t_filter_w)
	#I_Goal = Z.Z_BandStopFilter(Z.Z_Gain(Z.Z_PID(kvw, kpw, tpw, kdw, satw, W_Goal, Way_G, V_Goal, V_G), -1.0), 0.05, 0.0005)

	D_I_Goal= Z.Z_Derivative(I_Goal)
	
	Ecart_D_I = Z.Z_Sum(D_I_Goal, Gyro, ksa/100.0, -ksa/100.0)

	# All about orientation
	Orientation = Z.Z_Filter(Z.Z_Sensor(localisation.get_teta), 0.1)
	D_Orientation = Z.Z_Filter(Z.Z_Derivative(Orientation), 0.5)
	Teta_Goal = Z.Z_Constant(0.0)
	D_Teta_Goal = Z.Z_Derivative(Teta_Goal)
	
	Dir   = Z.Z_BandStopFilter(Z.Z_Gain(Z.Z_PID(kvo, kpo, tpo, kdo, sato*U_alim, Teta_Goal, Orientation, D_Teta_Goal, D_Orientation), 1/U_alim), 1.2, 0.1)
	#Dir   = Z.Z_Filter(Z.Z_Gain(Z.Z_PID(kvo, kpo, tpo, kdo, sato*U_alim, Teta_Goal, Orientation, D_Teta_Goal, D_Orientation), 1/U_alim), t_filter_o)
	#Motor = Z.Z_Filter(Z.Z_Sum(Z.Z_PID(kva, kpa, tpa, kda, sata*U_alim, I_Goal, Estimated_Incl, D_I_Goal, Gyro), D_Gyro, 1/U_alim, -kaa/U_alim), t_filter_a)
	Motor = Z.Z_BandStopFilter(Z.Z_Sum(Z.Z_PID(kva, kpa, tpa, kda, sata*U_alim, I_Goal, Estimated_Incl, Ecart_D_I, Z.Z_Constant(0.0)), D_Gyro, 1/U_alim, -kaa/U_alim), 0.1, 0.001) #0.1 0.008
	#Motor = Z.Z_Sum(Z.Z_PID(kva, kpa, tpa, kda, sata*U_alim, I_Goal, Estimated_Incl, D_I_Goal, Gyro), D_Gyro, 1/U_alim, -kaa/U_alim)
	
	i = 0	
	max = 0.0
	min = 10.0
	while (time.time()-t_begin_program  < time_prgm):
		i = i +1
		#W_Goal.set_val(0.05 * (1 - math.cos((time.time()-t_begin_program)*6.28*0.1)))
#		Teta_Goal.set_val((time.time()-t_begin_program)*0.4)
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
		i2cdevice.set_speed( m-d, m+d)

		if i > 100:
			print("w: {0}\tv: {1}\tI_soll: {2}\tWgoal: {3} ".format(Way_G.get_val(), V_G.get_val(), I_Goal.get_val(), W_Goal.get_val()))
			i = 0
		Z.Z_Index += 1
		if edit_file:
			my_file.write("{0}\t{1}\t{2}\t{3}\t{4}\n".format(time.time()-t_begin_program, i2cdevice.get_gyro_x(), i2cdevice.get_estimated_incl(),  m/U_alim, localisation.get_way()))

		#print(time.time()-t_begin_program, "\t", Orientation.get_val())
		t2 = loop_time + t_begin_loop - time.time()
#		print (Estimated_Incl.get_val())
#		print (t2)
		
		if (t2>max) : max = t2
		if (t2<min) : min = t2
		if t2<0:
			t2=0
		time.sleep(t2)
	print("min: ", min, " ; max: ", max)
	i2cdevice.set_speed(0,0)
	# End go_robot_go

Continue = True
while Continue:
	mode = input("Quit: q ; Continue: c\n$ ")
	if (mode == "q" or mode == "Q"):
		Continue = False	
	else:
		go_robot_go(myI2cDev, myLoc, 50)
		
import RPi.GPIO as GPIO
GPIO.cleanup()
if edit_file:
	my_file.close()

myLoc.finish()
myI2cDev.finish()
del myLoc
del myI2cDev
time.sleep(0.1)
sys.exit()
