#!/usr/bin/python3.4
# -*-coding:utf-8 -*

from constants import *
import RPi.GPIO as GPIO
import math
from time import time, sleep
from threading import Thread

#GPIO.cleanup()
GPIO.setmode(GPIO.BCM) # use hardware pin numbers
GPIO.setwarnings(True)

class Encoder():
	# This class could be use by itself, in case you have only one quadrature encoder
	# The point is to minimise calculation wihtin the interruptions in order not to miss any

	def changeA(self, term):
		self.A = GPIO.input(self.pinA)
		if self.A:  # risinging edge on A
			if self.B:
				self.count -= 1
			else:
				self.count += 1
		else:	# fallinging edge on A
			if self.B:
				self.count += 1
			else:
				self.count -= 1

	def changeB(self, term):
		self.B = GPIO.input(self.pinB)
		if self.B:  # risinging edge on B
			if self.A:
				self.count += 1
			else:
				self.count -= 1
		else:	# falling edge on B
			if self.A:
				self.count -= 1
			else:
				self.count += 1

	def __init__(self, pinA, pinB):
		self.pinA = pinA
		self.pinB = pinB
		self.count = 0
		self.erreurs= 0
		GPIO.setup(pinA, GPIO.IN, pull_up_down=GPIO.PUD_UP) # pull_up_down=GPIO.PUD_UP
		GPIO.setup(pinB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		self.A = GPIO.input(pinA)
		self.B = GPIO.input(pinB)
		done = False
		# Setting the interuptions. Sometimes it fails, so try again
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
		self.A = GPIO.input(pinA)
		self.B = GPIO.input(pinB)

	# this is the getter you wanna use for sensor displacement
	def get_count(self):
		c = self.count
		self.count = 0
		return c

# Create an item of the loc class to get track of you position
# The run() method must be called frequently
class loc():
	def __init__(self):
		self.left_speed = self.right_speed = self.pos_speed = 0.0
		self.pos_x = self.pos_y = self.pos_teta = self.pos_way = 0.0
		self.Enc_Right= Encoder(pinRightA,pinRightB)
		self.Enc_Left = Encoder(pinLeftA, pinLeftB)
		self.count_l = self.count_r = 0
		self.dxLeft = dxLeft
		self.dxRight= dxRight
		self.dtetaLeft = math.atan(dxLeft /(2.0*wheels_width))
		self.dtetaRight= math.atan(dxRight/(2.0*wheels_width))
		self.t_old = time()

	def update(self):
		self.count_l = - self.Enc_Left.get_count()
		self.count_r =  self.Enc_Right.get_count()
		#print(self.count_l, ";\t", self.count_r)
		t = time()
		#self.r += self.count_r
		#self.l += self.count_l
		self.left_speed = self.dxLeft * self.count_l / (t - self.t_old)
		self.right_speed= self.dxRight* self.count_r / (t - self.t_old)
		self.pos_speed = (self.left_speed + self.right_speed) / 2.0
		self.t_old = t

		if (self.count_l != 0 or self.count_r != 0):
			d_way = (self.count_l * self.dxLeft + self.count_r * self.dxRight) / 2.0
			self.pos_way += d_way
			half_d_teta  = - self.count_l * self.dtetaLeft + self.count_r * self.dtetaRight
			self.pos_teta += half_d_teta
			self.pos_x += d_way * math.cos(self.pos_teta)
			self.pos_y += d_way * math.sin(self.pos_teta)
			self.pos_teta += half_d_teta
			#self.pos_errors = self.Enc_Right.erreur + self.Enc_Left.erreur

	def get_teta(self):
		return (self.pos_teta)

	def get_speed(self):
		return (self.pos_speed)

	def get_left_speed(self):
		return (self.left_speed)

	def get_right_speed(self):
		return (self.right_speed)

	def get_x(self):
		return(self.pos_x)

	def get_y(self):
		return(self.pos_y)

	def get_way(self):
		return(self.pos_way)

#End class loc



if __name__ == "__main__":
	import pwmMotors
	pwmMotors = pwmMotors.PWM(0x40)
	pwmMotors.set_speed(0,0)
	MyLoc = loc()

	def test_kv(Motors):
		def test_value(value_pwm):
			Motors.set_speed(value_pwm, value_pwm)
			sleep(0.3)
			t = time()
			sum_l = sum_r = 0.0
			n = 0
			MyLoc.update()
			while (time()-t < 0.7):
				n+=1
				MyLoc.update()
				sum_l += MyLoc.get_left_speed()
				sum_r += MyLoc.get_right_speed()
				sleep (0.05)
			sum_l /= n
			sum_r /= n
			print (value_pwm, ";\t", sum_l, ",\t", sum_r, ";\t", n)
			
		for i in range(0, 90, 5):
			test_value(i)
		for i in range(90, -90 , -5):
			test_value(i)
		for i in range(-90, 0, 5):
			test_value(i)

	mla = mra = mlb = mrb = 0.0
	x = 0
	print ("pos_x \t pos_Y \t pos_teta \t pos_speed")
#	while(not (x == 'q' or x == 'Q')):
	while(False):
#		pwmMotors.set_speed(x, x)
		MyLoc.update()
#		print( "l: ", l, "\tr: ", r)
#		print (MyLoc.pos_x,"\t", MyLoc.pos_y,"\t", MyLoc.pos_teta,"\t", MyLoc.pos_speed)
		#mla = 0.99 * mla + 0.01 * GPIO.input(pinLeftA)
		#mra = 0.99 * mra + 0.01 * GPIO.input(pinRightA)
		#mlb = 0.99 * mlb + 0.01 * GPIO.input(pinLeftB)
		#mrb = 0.99 * mrb + 0.01 * GPIO.input(pinRightB)
#		print(mla,"\t",  mlb,"\t", mra,"\t", mrb) 
#		print(GPIO.input(pinLeftA),GPIO.input(pinLeftB),GPIO.input(pinRightA),GPIO.input(pinRightB))
#		x = keyboard.read(100, timeout = 0)
		sleep(0.005)
		x += 1

	test_kv(pwmMotors)

	while math.copysign(x, 1) > 10:
		pwmMotors.set_speed(0, x)
		MyLoc.update()
		x -= math.compysign( 1, x)
		sleep(0.01)
	pwmMotors.set_speed(0,0)
