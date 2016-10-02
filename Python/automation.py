#!/usr/bin/python3.4
# -*-coding:utf-8 -*

import math
import time
import subprocess

Z_Index = 0 # This integer must be incremented for each calculation cycle

class Z_Constant:
	def __init__(self, valeur_init = 0.0):
		self._valeur = valeur_init
		self._timer0 = time.time()
		self._local_index = 0

	def _update(self):
		# well, if this is a constante, bet you'll do nothing
		# print ("Z_Constante._update")
		pass

	def get_val(self):
		global Z_Index
		if (self._local_index < Z_Index):
			self._update()
			self._timer0 = time.time()
			self._local_index = Z_Index
		return self._valeur
	
	# use with responsability
	def set_val(self, n_val):
		self._valeur = n_val

class Z_Sensor(Z_Constant):
	# use this class when you've got you pick up a value from a function or an application
	def __init__(self, get_sensor):
		Z_Constant.__init__(self, 0.0 )
		self._ref_get_sensor = get_sensor

	def _update(self):
		self._valeur = self._ref_get_sensor()
		#print("update sensor")


class Z_Sum(Z_Constant):
	def __init__(self, Z_Item1, Z_Item2, factor1 = 1.0, factor2 = 1.0):
		Z_Constant.__init__(self)
		self._Item1 = Z_Item1
		self._Item2 = Z_Item2
		self._f1 = factor1
		self._f2 = factor2

	def _update(self):
		self._valeur = self._f1 * self._Item1.get_val() + self._f2 * self._Item2.get_val()

class Z_Filter(Z_Constant):
	# useful to get a better reading from a noisy sensor. Band_pass argument is a time in secondes. Choose wisely
	def __init__(self, Z_item, band_pass = 1.0 , valeur_init = 0.0):
		Z_Constant.__init__(self, valeur_init)
		try:
			if not isinstance(Z_item, Z_Constant):
				raise TypeError("Not a instance of Z_Constante")
		except:
			print("Invalid argument, objet of type Z_ required")
		self._Z_Item = Z_item  # reference to an objet wich inherits from Z_Constant
		self._memorie = 0.0
		self._t_cut = band_pass
		
	def _update(self):
		if self._t_cut == 0:
			self._valeur = self._Z_Item.get_val()
			return
		 # first order linear filter
		val = self._Z_Item.get_val()
		t = time.time()
		#print ("Z_filter.update, val: ", val )
		self._memorie = (self._memorie + val * ( t - self._timer0) / self._t_cut) / ( 1 + ( t - self._timer0 ) / self._t_cut)
		self._valeur = self._memorie

class Z_Gain(Z_Filter):
	def _update(self):
		self._valeur = self._t_cut * self._Z_Item.get_val()


class Z_HighPassFilter(Z_Filter):
	def _update(self):
		val = self._Z_Item.get_val()
		self._valeur = (self._valeur + val - self._memorie) * self._t_cut / ( self._t_cut + time.time() - self._timer0)
		self._memorie = val

class Z_BandStopFilter(Z_Constant):
	def __init__(self, Z_item, t_low, t_high):
		Z_Constant.__init__(self, 0.0)
		self.lowPass = Z_Filter(Z_item, t_low)
		self.highPass= Z_HighPassFilter(Z_item, t_high)
		
	def _update(self):
		self._valeur = self.lowPass.get_val() + self.highPass.get_val()

class Z_BandPassFilter(Z_Filter):
	def __init__(self, Z_item, t_low, t_high):
		Z_Constant.__init__(self, 0.0)
		self.highPass= Z_HighPassFilter(Z_item, t_high)
		self.lowPass = Z_Filter(self.highPass, t_low)
		
	def _update(self):
		self._valeur = self.lowPass.get_val()
		


class Z_Derivative(Z_Filter):
	# Oh yeah, Z_Speed = Z_Derivative( my_Z_position ) Awsome right?
	def __init__(self, Z_item):
		Z_Filter.__init__(self, Z_item)
		
	def _update(self):
		self._valeur = (self._Z_Item.get_val() - self._memorie) / ( time.time() - self._timer0)
		self._memorie = self._Z_Item.get_val()


class Z_Integral(Z_Filter):
	def _update(self):
		dt = time.time() - self._timer0
		self._valeur += self._Z_Item.get_val() * dt
		if self._t_cut > 0:
			self._valeur = self._valeur *  self._t_cut / ( self._t_cut + dt)

		

class Z_PID(Z_Filter):
	# Once you've use a PID controller, it all seems fairly natural. Otherways, it's kind of complicated
	# I propose hier a state of the art PID algorithm with posibility of speed pre-control surch as thoses
	# used in actuals industrial robots, mills... You can still improve by adding a band-stop but that's kind of fancy
	# You want to control a position, a temperature, a force?
	# This algorithm compares the value you want, the value from the sensor (and their derivatives),
	# and decide the adapted electrical voltage (or whaterver energy you use) for you actuator
	def __init__(self, kv, kp, tp, kd, sat, Z_Goal, Z_Sensor = Z_Constant(0.0), Z_D_Goal = Z_Constant(0.), Z_D_Sensor = Z_Constant(0.)):
		try:	
			if (not isinstance(Z_Goal, Z_Constant) and not isinstance(Z_Sensor, Z_Constant) and not isinstance(Z_D_Goal, Z_Constant) and not isinstance(Z_D_Sensor, Z_Constant)):
				raise TypeError("Not a instance of Z_Constante")
		except:
			print("Invalid argument, objet of type Z_ required")
		Z_Filter.__init__(self, Z_Goal)
		self._memorie = 0.0
		self._ecart_old = 0.0
		self._Sensor = Z_Sensor
		self._D_Sensor = Z_D_Sensor
		self._D_Goal = Z_D_Goal
		self._kv = kv
		self._kp = kp
		self._kd = kd
		if not tp > 0:
			self._tp = 1000.0
		else:
			self._tp = tp
		if not sat > 0:
			self._sat = 50.0
		else:
			self._sat = sat

	def _update(self):
		dt = time.time() - self._timer0
		ecart =  self._kv *  (self._Z_Item.get_val() - self._Sensor.get_val())
		ecart += (self._D_Goal.get_val() - self._D_Sensor.get_val())
		self._memorie =+ ecart * dt / self._tp
		# The Integral part (memorie) can't exed 50% of the max value (sat) It would suppose a problem anyway
		if self._memorie > 0.5 * self._sat/self._kp:
			self._memorie = 0.5 * self._sat/self._kp
		elif self._memorie < -0.5 * self._sat/self._kp:
			self._memorie = -0.5 * self._sat/self._kp

		self._valeur = ecart * self._kp + self._memorie + (ecart - self._ecart_old) * self._kd / dt
		self._ecart_old = ecart
		
		# Saturation :
		if self._valeur > self._sat:
			self._valeur = self._sat
		elif self._valeur < -self._sat:
			self._valeur = - self._sat

		#print("pid , dt: ", dt, "|t ecart :", ecart, "\t valeur :", self._valeur)
		
#test du module
if __name__ == "__main__":
	Zero = Z_Constant(0)
	Un = Z_Constant(0.0)
	Filtre = Z_Filter(Zero, 0.2)
	Deriv = Z_Derivative(Filtre)
	pid = Z_PID (10, 15, 10, 12, 100, Un, Filtre, Deriv, Zero)
	somme = Z_Sum(pid, Deriv)
	sensor = Z_Sensor(Deriv.get_val)
	for i in range(1, 200, 1):
		Z_Index += 1
		ang = math.sin(i/10)
		Zero.set_val(ang)
		print ("val: ", Zero.get_val(), "\t filtre: ", Filtre.get_val(), " \t Derivee: ", Deriv.get_val(), "\t  pid: ", pid.get_val(), " \t somme: ", somme.get_val())
		time.sleep(0.01)
		
