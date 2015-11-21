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
		pass

	def get_val(self):
		global Z_Index
		if (self._local_index < Z_Index):
			self.update()
			self._timer0 = time.time()
			self._local_index = Z_Index
		return self._valeur
	
	# use with responsability
	def set_val(n_val):
		self._valeur = n_val


class Z_Filter(Z_Constant):
	# useful to get a better reading from a noisy sensor. Band_pass argument is a time in secondes. Choose wisely
	def __init__(self, Z_item, band_pass = 1.0 , valeur_init = 0.0):
		Z_Constant.__init__(self, valeur_init)
		try:
			if not isinstance(Z_item, Z_Constant):
				raise TypeError("Not a instance of Z_Constante")
		except:
			print("Invalid argument, objet of type Z_ required")
		self.Z_Item = Z_item  # reference to an objet wich inherits from Z_Constant
		self._memorie = 0.0
		try:
			if band_pass > 0:
				self._t_cut = band_pass
		except:
			print("Invalid argument, positive band_pass value")


	def _update(self):
		 # first order linear filter
		val = self._Z_Item.get_val()
		t = time.time()

		self.memorie += (t-self._timer0) / self._t_cut  * val
		
#test du module
if __name__ == "__main__":
	Zero = Z_Constant(0)
	Filtre = Z_Filter(Zero, 0.2)
	for i in range(1, 1000, 1):
		Z_Index += 1
		Zero.set_val(math.sin(i/10))
		print ("val: ", Zero.get_val(), "\t filtre: ", Filtre.get_val())
		time.sleep(0.01)
		
