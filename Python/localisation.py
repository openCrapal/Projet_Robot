#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import RPi.GPIO as GPIO
import math
from time import time, sleep

#GPIO.cleanup()
GPIO.setmode(GPIO.BCM) # use hardware pin numbers
GPIO.setwarnings(True)

#Pins I use
pinLeftA = 14
pinLeftB = 15
pinRightA= 18
pinRightB= 23

#Global localisition variables
pos_x = 0.0
pos_y = 0.0
pos_teta = 0.0    #orientation in rads
pos_speed = 0.0   #axial speed

pos_errors = 0
timer = time()

# physical dimentions of the car
dxLeft = 0.042*3.141/48
dxRight= 0.042*3.141/48
wheels_width = 0.128   # distance beetwin wheels

class Encoder():

	def changeA(self, term):
		newA = self.INPUT(self.pinA)
		if self.A == newA:
			self.erreur +=1
		elif newA:  # rising edge on A
			if self.B:
				self.count -= 1
			else:
				self.count += 1
		else:
			if self.B:
				self.count += 1
			else:
				self.count -= 1
		self.A = newA

	def changeB(self, term):
		newB = self.INPUT(self.pinB)
		if newB == self.B:
			self.erreur += 1
		elif newB:  # rising edge on B
			if self.A:
				self.count += 1
			else:
				self.count -= 1
		else:
			if self.A:
				self.count -= 1
			else:
				self.count += 1
		self.B = newB

	def fallB(self, term):
		if self.A:
			self.count -= 1
		else:
			self.count += 1
		self.B = False


	def __init__(self, pinA, pinB):
		self.INPUT = GPIO.input
		self.pinA = pinA
		self.pinB = pinB
		self.count = 0
		self.erreur= 0
		GPIO.setup(pinA, GPIO.IN, pull_up_down=GPIO.PUD_UP) # pull_up_down=GPIO.PUD_UP
		GPIO.setup(pinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		self.A = GPIO.input(pinA)
		self.B = GPIO.input(pinA)
		done = False
		while not done:
			try:
				GPIO.add_event_detect(pinA, GPIO.BOTH, callback = self.changeA) 
			except:
				pass
			else:
				done = True
		done = False
		while not done:
			try:
				GPIO.add_event_detect(pinB, GPIO.BOTH, callback = self.changeB ) 
			except:
				pass
			else:
				done = True

	def get_count(self):
		c = self.count
		self.count = 0
		return c

# Declaration of the two object "Encoders" used of localisation
Enc_Left = Encoder(pinLeftA, pinLeftB)
Enc_Right= Encoder(pinRightA,pinRightB)


# I must have done too much c++, there must be a more pythonic way to do that
def get_teta():
	# update()
	return (pos_teta)

def get_speed():
	return (pos_speed)

def get_x():
	return pos_x

def get_y():
	return pos_y

# To save time during interruptions, the "complex" trigonometric calculation is
# only done when the user need it
# if you don't call it often enought, you might have some big localisation errors
def update():
	global timer
	t = time()
	global pos_x, pos_y, pos_teta, pos_speed
	count_l = Enc_Left.get_count()
	count_r = Enc_Right.get_count()
	d_way = (count_l * dxLeft + count_r * dxRight) / 2.0
	pos_speed = d_way / ( t - timer )
	half_d_teta  = count_l * math.atan(dxLeft/2.0/wheels_width) - count_r * math.atan(dxRight/2.0/wheels_width)
	pos_teta -= half_d_teta
	pos_x += d_way * math.cos(pos_teta)
	pos_y += d_way * math.sin(pos_teta)
	pos_teta -= half_d_teta
	timer = t
	pos_errors = Enc_Right.erreur + Enc_Left.erreur

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
	while(i<200):
		update()
		print (pos_x, pos_y, pos_teta, pos_speed, pos_errors)
		i += 1
		sleep(0.01)

