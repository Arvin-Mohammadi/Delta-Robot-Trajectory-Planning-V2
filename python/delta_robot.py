
# =================================================================================================
# -- IMPORTS --------------------------------------------------------------------------------------
# =================================================================================================

import numpy as np 
import matplotlib.pyplot as plt 
import time 
import math 
from numpy import sin, cos, tan, pi


# =================================================================================================
# -- functions  --------------------------------------------------------------------------------------
# =================================================================================================

def tand(theta):
	return tan(theta*pi/180)

def sind(theta):
	return sin(theta*pi/180)

def cosd(theta):
	return cos(theta*pi/180)

# =================================================================================================
# -- Delta Robot Class ----------------------------------------------------------------------------
# =================================================================================================

class DeltaRobot: 
	def __init__(self, rod_b, rod_ee, r_b, r_ee):
		# configes the robot 

		self.rod_b = rod_b
		self.rod_ee = rod_ee
		self.r_b = r_b
		self.r_ee = r_ee
		self.alpha = np.array([0, 120, 240])


	def forward_kin(self, theta):
		# calculate FK, takes theta(deg)

		rod_b = self.rod_b
		rod_ee = self.rod_ee

		theta = np.array(theta) 

		theta1 = theta[0]
		theta2 = theta[1]
		theta3 = theta[2]

		side_ee	= 2/tand(30)*self.r_ee 
		side_b 	= 2/tand(30)*self.r_b

		t = (side_b - side_ee)*tand(30)/2

		y1 = -(t + rod_b*cosd(theta1))
		z1 = -rod_b*sind(theta1)

		y2 = (t + rod_b*cosd(theta2))*sind(30)
		x2 = y2*tand(60)
		z2 = -rod_b*sind(theta2)

		y3 = (t + rod_b*cosd(theta3))*sind(30)
		x3 = -y3*tand(60)
		z3 = -rod_b*sind(theta3)

		dnm = (y2 - y1)*x3 - (y3 - y1)*x2

		w1 = y1**2 + z1**2
		w2 = x2**2 + y2**2 + z2**2
		w3 = x3**2 + y3**2 + z3**2 

		a1 = (z2-z1)*(y3-y1) - (z3-z1)*(y2-y1)
		b1 = -((w2-w1)*(y3-y1) - (w3-w1)*(y2-y1))/2

		a2 = -(z2-z1)*x3 + (z3-z1)*x2
		b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2

		a = a1**2 + a2**2 + dnm**2
		b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm**2)
		c = (b2 - y1*dnm)**2 + b1**2 + dnm**2*(z1**2 - rod_ee**2)

		d = b**2 - 4*a*c
		if d < 0:
			return -1 

		z0 = -0.5*(b + d**0.5)/a
		x0 = (a1*z0 + b1)/dnm
		y0 = (a2*z0 + b2)/dnm

		return np.array([x0, y0, z0])

	def inverse_kin(self, _3d_pose):
		# calculates IK, returns theta(deg)
		[x0, y0, z0] = _3d_pose

		rod_ee = self.rod_ee
		rod_b = self.rod_b
		r_ee = self.r_ee
		r_b = self.r_b 
		alpha = self.alpha 

		F1_pos = ([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
		J1_pos = ([[0, 0, 0], [0, 0, 0], [0, 0, 0]])
		theta = [0, 0, 0]

		for i in [0, 1, 2]:

			x = x0*cosd(alpha[i]) + y0*sind(alpha[i])
			y = -x0*sind(alpha[i]) + y0*cosd(alpha[i])
			z = z0

			ee_pos = np.array([x, y, z])

			E1_pos = ee_pos + np.array([0, -r_ee, 0])
			E1_prime_pos = np.array([0, E1_pos[1], E1_pos[2]])
			F1_pos[i] = np.array([0, -r_b, 0])

			_x0 = E1_pos[0]
			_y0 = E1_pos[1]
			_z0 = E1_pos[2]
			_yf = F1_pos[i][1]

			c1 = (_x0**2 + _y0**2 + _z0**2 + rod_b**2 - rod_ee**2 - _yf**2)/(2*_z0)
			c2 = (_yf - _y0)/_z0
			c3 = -(c1 + c2*_yf)**2 + (c2**2+ 1)*rod_b**2

			if c3 < 0:
				# print("non existing point")
				return int(-1)

			J1_y = (_yf - c1*c2 - c3**0.5)/(c2**2 + 1)
			J1_z = c1 + c2*J1_y
			F1_y = -r_b

			theta[i] = math.atan(-J1_z/(F1_y - J1_y))*180/pi

		return np.array(theta)


# =================================================================================================
# -- main loop ----------------------------------------------------------------------------
# =================================================================================================
# FK: check
# IK: check

if __name__ == "__main__":

	# test1 
	delta = DeltaRobot(0.2, 0.46, 0.1, 0.074)
	inv = delta.inverse_kin( [0, -0.15, -0.42])
	print(inv)
