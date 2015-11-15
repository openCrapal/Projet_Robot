#!/usr/bin/python3.4

import RPi.GPIO as GPIO
import math
import time

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
pos_teta = 0.0  #orientation in rads
pos_way = 0.0   #absolute distance the car went throught

pos_errors = 0

# physical dimentions of the car
dxLeft = 0.042*3.141/48
dxRight= 0.042*3.141/48
wheels_width = 0.128   # distance beetwin wheels

GPIO.setmode(GPIO.BCM) #use hardware pin numbers


# As the localisation begins,this  module should be initialised
def init():
	global pinLeftA,  pinLeftB, pinRightA, pinRightB
	global EncoderLeftA_old, EncoderLeftB_old, EncoderRightA_old, EncoderRightB_old

	pos_x = 0.0
	pos_y = 0.0
	pos_teta = 0.0

	GPIO.setwarnings(True)       

	GPIO.setup(pinLeftA,  GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  
	GPIO.setup(pinLeftB,  GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  
	GPIO.setup(pinRightA, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  
	GPIO.setup(pinRightB, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

	L_A_old = GPIO.input(pinLeftA)
	L_B_old = GPIO.input(pinLeftB)
	R_A_old= GPIO.input(pinRightA)
	R_B_old= GPIO.input(pinRightB)
	
	pos_x = 0.0
	pos_y = 0.0
	pos_teta = 0.0
	
	# Initialize the interrupts - these trigger on the both the rising and falling 
	GPIO.add_event_detect(pinLeftA, GPIO.BOTH, callback = leftEncoder)   # Encoder A
	GPIO.add_event_detect(pinLeftB, GPIO.BOTH, callback = leftEncoder)   # Encoder B
	GPIO.add_event_detect(pinRightA, GPIO.BOTH, callback = rightEncoder)   # Encoder A
	GPIO.add_event_detect(pinRightB, GPIO.BOTH, callback = rightEncoder)   # Encoder B


# Define encoder count function
def leftEncoder(term):
	counts = 0
	global  L_B_old, L_A_old, pos_way, pos_x, pos_y, pos_teta, pos_errors
	A = GPIO.input(pinLeftA)  # stores the value of the encoders at time of interrupt
	B = GPIO.input(pinLeftB)

	if (not A and not B and not L_A_old and L_B_old or not A and B and L_A_old and L_B_old or A and B and L_A_old and not L_B_old or A and not B and not L_A_old and not L_B_old):
		# this will be clockwise rotation
		counts = 1


	elif (not A and B and not L_A_old and not L_B_old or A and B and not L_A_old and L_B_old or A and not B and L_A_old and L_B_old or not A and not B and L_A_old and not L_B_old):
		# this will be counter-clockwise rotation
		counts = -1
		#print 'Encoder count is %s' %counts
		#print 'AB is %s %s' % (Encoder_A, Encoder_B)


	else:
	#this will be an error
		pos_errors += 1
		print ('Error encoder left. Error count is ', pos_errors)
		L_A_old = A
		L_B_old = B
		return


	L_A_old = A     # store the current encoder values as old values to be used as comparison in the next loop
	L_B_old = B
	
	pos_way += math.fabs(counts) * dxLeft/2.0
	half_d_teta  = counts * math.atan(dxLeft/2.0/wheels_width)
	pos_teta -= half_d_teta
	pos_x += counts * dxLeft*0.5*math.cos(pos_teta)
	pos_y += counts * dxLeft*0.5*math.sin(pos_teta)
	pos_teta -= half_d_teta

def rightEncoder(term):
	counts = 0
	global  R_B_old, R_A_old, pos_way, pos_x, pos_y, pos_teta, pos_errors
	A = GPIO.input(pinRightA)  # stores the value of the encoders at time of interrupt
	B = GPIO.input(pinRightB)

	if (not A and not B and not R_A_old and R_B_old or not A and B and R_A_old and R_B_old or A and B and R_A_old and not R_B_old or A and not B and not R_A_old and not R_B_old):
		#this will be counter-clockwise rotation
		counts = -1
		#print 'Encoder count is %s' %counts



	elif (not A and B and not R_A_old and not R_B_old or A and B and not R_A_old and R_B_old or A and not B and R_A_old and R_B_old or not A and not B and R_A_old and not R_B_old):
		# this will be clockwise rotation
		counts = 1

	else:
        #this will be an error
		pos_errors += 1
		print ('Error encoder Right. Error count is ' , pos_errors)
		R_A_old = A
		R_B_old = B
		return


	R_A_old = A     # store the current encoder values as old values to be used as comparison in the next loop
	R_B_old = B
	
	pos_way += math.fabs(counts) * dxRight/2.0
	half_d_teta  = counts * math.atan(dxRight/2.0/wheels_width)
	pos_teta += half_d_teta
	pos_x += counts * dxRight*0.5*math.cos(pos_teta)
	pos_y += counts * dxRight*0.5*math.sin(pos_teta)
	pos_teta += half_d_teta



if __name__ == "__main__":
	init()
	i = 0
	print ("pos_x \t pos_Y \t pos_teta \t pos_way")
	while(i<100):
		print (pos_x, pos_y, pos_teta, pos_way)
		i += 1
		time.sleep(0.2)
	GPIO.cleanup()

