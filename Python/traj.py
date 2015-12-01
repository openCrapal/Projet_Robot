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
		for i in range(2 , self._size+1, 1):
			self._aj.append(self._aj[i-1] * tau / i) 
 
	def get_traj(self, t):
		traj = list()
		for i in range(self._size):
			traj.append(0.0)
		if t >= self._t0 + self._tau:
			t1 = t - self._t0 - self._tau
			for i in range(1, self._size, 1):
				traj[i] = 0.0
				for j in range (0, i, 1):
					traj[i] += self._aj[i-j] * math.pow(t1,j) / math.factorial(j)  
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
		self._maxis = list()
		for i in range(degree + 1):
			self._pos.append(0.0)
			try:
				self._maxis.append(list_of_maxis[i])
			except:
				print("max value of rang {0} not defined. setting to 1.0".format(i))
				self._maxis.append(1.0)
				pass
		self._phases = list()
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
		#print(self._pos)

	def _generator(self, ampl, tau, pos_init, to=time.time(), t_middle=0.0):
		table = [1]
		for i in range(1, self._degree, 1):
			n =  len(table)
			for j in range(0, n, 1):
				table.append(-table[j])
		n = len(self._phases)
		for i in range(n):
			del(self._phases[0])
		for i in range(0, len(table)//2, 1):
			self._phases.append(Phase(table[i]*ampl, tau, to + i * tau, self._degree))
		for i in range(len(table)//2, len(table), 1):
			self._phases.append(Phase(table[i]*ampl, tau, t_middle + to + i * tau, self._degree))

	#set the traj generator to the fastest way from pos_init to goal
	#both are steal standing points !!!
	#default pos_init is the point you were, but you can reset it
	def set_goal(self, goal, to=time.time()):
		# find the fastest way
		list_tau = list()
		for i in range(1, self._degree+1 , 1):
			B = 1.0
			if i == 3:
				B = 2.0
			elif i==4:
				B = 8.0
			elif i ==5:
				B = 32.0
			elif i ==6:
				B = 32.0*8.0
			elif i ==7:
				B = 32.0*8.0*16.0
			list_tau.append(pow( math.fabs(goal) / (B * self._maxis[i]), 1/i))
		print(list_tau)
		tau = max(list_tau)
		tau = max(tau, 0.1)
		ampl = 2.0 * goal / ( pow(2.0 * tau, self._degree))
		print ("tau: ", tau, " ;ampl: ", ampl)
		#test
		self._generator(ampl, tau, self._pos_init, to)
		tableau_maxs = list()
		#for i in range(1, self._degree+1):
		#	self.update(pow(2, i+1) * tau)
		#	tableau_maxs.append(self._pos[i])
		#print (goal, tableau_maxs)

#test du module
if __name__ == "__main__":
	maxis = [1000.0, 100.0, 3., 1.0, 1000.0]
	ma_phase = Phase(-1.0, 2.0, 1.0, 5)
	ma_traj = Traj(4, maxis)
	print("degree4")
	ma_traj.set_goal(10.0, 0.0)
	for i in range(0, 201, 1):
		ma_traj.update(i/10.)
		print(i/10.0, ", ", ma_traj._pos)

