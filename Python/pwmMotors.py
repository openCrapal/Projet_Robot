#!/usr/bin/python3.4
# -*-coding:utf-8 -*

# Small module powering two motors throught Cyclon 2ways 10A MotorDriverCard
# Use RPi.GPIO for both pwm and direction pin

import RPi.GPIO as GPIO
import math
import time

# Pins I use on my RPy rev2 model B
DIR1 = 22  # Right
PWM1 = 27
DIR2 = 17  # Left, obviously
PWM2 = 4

GPIO.setmode(GPIO.BCM) #use hardware pin numbers


GPIO.setup(DIR1,  GPIO.OUT)
GPIO.setup(PWM1,  GPIO.OUT)
GPIO.setup(DIR2,  GPIO.OUT)
GPIO.setup(PWM2,  GPIO.OUT)

GPIO.output(DIR1, 0)
GPIO.output(DIR2, 0)
p1 = GPIO.PWM(PWM1, 500)
p2 = GPIO.PWM(PWM2, 500)
p1.start(0)
p2.start(0)

def set_speed(v1, v2):
	dir1 = 0
	if v1>=0:
		if v1>=100:
			v1=100
		dir1=0
	elif v1<0:
		if v1<= -100:
			v1=-100
		v1 = -v1
		dir1=1

	dir2 = 0
	if v2>=0:
		if v2>=100:
			v2=100
		dir2=0
	elif v2<0:
		if v2<= -100:
			v2=-100
		v2 = -v2
		dir2=1
	
	GPIO.output(DIR1, dir1)
	GPIO.output(DIR2, dir2)
	p1.ChangeDutyCycle(v1)
	p2.ChangeDutyCycle(v2)

def set_frequency(freq):
	if freq>=0:
		p1.ChangeFrequency(freq)
		p2.ChangeFrequency(freq)




if __name__ == "__main__":
	for i in range(0, 5, 1):
		set_speed(-10, float(input("Set speed between -100 and 100 : ")))
	GPIO.cleanup()

