#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import math
import time
#import subprocess

# Goal hier is to generate an position value that the controller can follow
# degree = n => the nth derivative exists, cad the (n-1)th has no edges
# 2 is usualy enought, 3 is fancy, I need degree 4 for a ballancing bot
# I'll use a math trick called (in France) superposition's theorem
# My path function will be the sum of the results of multiple accelerations phases 
class Phase:
	def __init__(self, ampl, tau, t0, degree = 3):
		self._size = degree + 1
		self._ampl = ampl
		self._tau = tau
		self._t0 = t0
		# [ xn+1, xn , ... , x1, x]
		self._aj = [0.0, ampl * tau]
		for i in range(2 , self._size, 1):
			self._aj.append(self._aj[i-1] * tau / i) 
 
	def get_traj(self, t):
		traj = list()
		for i in range(self._size):
			traj.append(0.0)
		if t >= self._t0 + self._tau:
			t1 = t - self._t0 - self._tau
			for i in range(1, self._size, 1):
				traj[i] = self._aj[i] + self._aj[i - 1] * t1
		elif t >= self._t0:
			t1 = t -self._t0
			traj[0] = self._ampl
			for i in range(1, self._size, 1):
				traj[i] = traj[i-1] * t1 / i
		return(traj)

class Traj:
	def __init__(self, degree, list_of_maxis=[1.0], pos_init = 0.0):
		self._degree = degree
		self._pos_init = pos_init
		self._pos = list()
		for i in range(degree + 1):
			self._pos.append(0.0)
		self._phases = list()
		#example
		self._phases.append(Phase(1.0, 1.0, 1.0, self._degree))
		self._phases.append(Phase(-1.0, 1.0, 2.0, self._degree))
	
	def update(self, t = time.time()):
		#clear
		for i in range(0, self._degree, 1):
			self._pos[i] = 0.0
		self._pos[-1] = self._pos_init
		for chaque_phase in self._phases:
			influss = chaque_phase.get_traj(t)
			for i in range(0, self._degree + 1, 1):
				self._pos[i] += influss[i]
		self._pos[-1] += self._pos_init
		#example
		print(self._pos)

#test du module
if __name__ == "__main__":
	ma_phase = Phase(-1.0, 2.0, 1.0, 5)
	ma_traj = Traj(5)
	for i in range(0, 50, 1):
		ma_traj.update(i/10.0)
		print("phase")
		print(ma_phase.get_traj(i/10.0))
