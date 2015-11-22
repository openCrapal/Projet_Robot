#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import RPi.GPIO as GPIO
import math
from time import time, sleep

pinLeftA = 14
pinLeftB = 15
pinRightA= 18
pinRightB= 23

R_A_old = 0
R_B_old = 0
L_A_old= 0
L_B_old= 0

pos_x = 0.0
pos_y = 0.0
pos_teta = 0.0    #orientation in rads
pos_speed = 0.0   #axial speed

pos_errors = 0
count_l = 0
count_r = 0
timer = time()

# physical dimentions of the car
dxLeft = 0.042*3.141/48
dxRight= 0.042*3.141/48
wheels_width = 0.128   # distance beetwin wheels

#GPIO.cleanup()
GPIO.setmode(GPIO.BCM) # use hardware pin numbers
GPIO.setwarnings(True)

GPIO.setup(pinLeftA,  GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(pinLeftB,  GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(pinRightA, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(pinRightB, GPIO.IN, pull_up_down=GPIO.PUD_UP)

L_A_old = GPIO.input(pinLeftA)
L_B_old = GPIO.input(pinLeftB)
R_A_old= GPIO.input(pinRightA)
R_B_old= GPIO.input(pinRightB)

# Define encoder count function
def leftEncoder(term):
	global L_B_old, L_A_old, pos_errors, count_l
	
	A = GPIO.input(pinLeftA)  # stores the value of the encoders at time of interrupt
	B = GPIO.input(pinLeftB)

	if (not A and not B and not L_A_old and L_B_old or not A and B and L_A_old and L_B_old or A and B and L_A_old and not L_B_old or A and not B and not L_A_old and not L_B_old):
		# this will be clockwise rotation
		count_l += 1


	elif (not A and B and not L_A_old and not L_B_old or A and B and not L_A_old and L_B_old or A and not B and L_A_old and L_B_old or not A and not B and L_A_old and not L_B_old):
		# this will be counter-clockwise rotation
		count_l -= 1
		#print 'Encoder count is %s' %counts
		#print 'AB is %s %s' % (Encoder_A, Encoder_B)


	else:
	#this will be an error
		pos_errors += 1
	
	L_A_old = A
	L_B_old = B # store the current encoder values as old values to be used as comparison in the next loop
	

def rightEncoder(term):
	global  R_B_old, R_A_old, pos_errors, count_r
	A = GPIO.input(pinRightA)  # stores the value of the encoders at time of interrupt
	B = GPIO.input(pinRightB)

	if (not A and not B and not R_A_old and R_B_old or not A and B and R_A_old and R_B_old or A and B and R_A_old and not R_B_old or A and not B and not R_A_old and not R_B_old):
		#this will be counter-clockwise rotation
		count_r -= 1
		#print 'Encoder count is %s' %counts


	elif (not A and B and not R_A_old and not R_B_old or A and B and not R_A_old and R_B_old or A and not B and R_A_old and R_B_old or not A and not B and R_A_old and not R_B_old):
		# this will be clockwise rotation
		count_r += 1

	else:
        #this will be an error
		pos_errors += 1
	
	R_A_old = A
	R_B_old = B
	

def update():
	global timer
	t = time()
	if (t - timer < 0.001):
			return

	global pos_x, pos_y, pos_teta, pos_speed, count_l, count_r
	d_way = (count_l * dxLeft + count_r * dxRight) / 2.0
	pos_speed = d_way / ( t - timer )
	half_d_teta  = count_l * math.atan(dxLeft/2.0/wheels_width) - count_r * math.atan(dxRight/2.0/wheels_width)
	pos_teta -= half_d_teta
	pos_x += d_way * math.cos(pos_teta)
	pos_y += d_way * math.sin(pos_teta)
	pos_teta -= half_d_teta
	timer = t
	count_l = count_r = 0

# Initialize the interrupts - these trigger on the both the rising and falling 
done = False
while (not done):
	try:
		GPIO.add_event_detect(pinRightA, GPIO.BOTH, callback = rightEncoder)   # Encoder A
	except:
		pass
	else:
		done = True

done = False
while (not done):
	try:
		GPIO.add_event_detect(pinRightB, GPIO.BOTH, callback = rightEncoder)   # Encoder B
	except:
		pass
	else:
		done = True

done = False
while (not done):
	try:
		GPIO.add_event_detect(pinLeftA, GPIO.BOTH, callback = leftEncoder)   # Encoder A
	except:
		pass
	else:
		done = True

done = False
while (not done):
	try:
		GPIO.add_event_detect(pinLeftB, GPIO.BOTH, callback = leftEncoder)   # Encoder B
	except:
		pass
	else:
		done = True


if __name__ == "__main__":
	import sys
	import signal

	def fermer_pgrm(signal, frame):
		print("fermer proprement")
		GPIO.cleanup()
		sys.exit(0)

	signal.signal(signal.SIGINT, fermer_pgrm)

	i = 0
	print ("pos_x \t pos_Y \t pos_teta \t pos_speed")
	while(i<100):
		update()
		print (pos_x, pos_y, pos_teta, pos_speed)
		i += 1
		sleep(0.2)

