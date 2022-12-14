# This is a new mehtod that i used for Jacobian matrix. The EE movement 
# is circular and the argument of "f" in x = cos(f) is equal to a(t^3 - 1.5Tt^2)


# =================================================================================================
# -- IMPORTS --------------------------------------------------------------------------------------
# =================================================================================================

# In this sectoin we import all of the needed dependencies 


import 	numpy as np 
from 	numpy import sin, cos
import 	math 
import 	matplotlib.pyplot as plt 


# =================================================================================================
# -- JACOBIAN MATRICES ----------------------------------------------------------------------------
# =================================================================================================

# in this section we calculate J_p and J_Theta_dot_dot from the input of: geometric features and 
# position of EE the entire purpose of the jacobian matrix is to relate the Theta_dot_dot matrix
# to EE velocity matrix 


class Jacobian:
	def __init__(self, EE_position, active_rod=0.2, passive_rod=0.46, base_side=0.3464101615, EE_radius=0.2563435195, alpha=[0, 120, 240]):
		## ***----------------*** THIS CLASS ALLOWS US TO FIND THE RELATION 
		## *** IMPORTANT NOTE *** BETWEEN VELOCITY AND ANGULAR VELOCITY 
		## ***----------------*** FOR ONE POINT IN 3D SPACE

		# EE and base sides are pointing to this: 
		# every joint attached to the EE make a triangle together. 
		# the side of that traingle is the number we have. 
		# (same for the base)

		# initializing the basic geometry and the given data
		self.alpha = np.array(alpha)						# alpha angles
		self.EE_position_global = np.array(EE_position)		# end effoctor position (x_e, y_e, z_e) with respect to alpha_0								
		self.active_rod = active_rod						# length of the active rod (the upper rod or r_f)
		self.passive_rod = passive_rod						# length of the passive rod (the lower rod or r_e)
		self.EE_radius = EE_radius							# side of EE traingle
		self.base_side = base_side							# side of base triangle 

	def get_theta_ij(self): 
		# this also calculate inverse kinematic

		# assigning constants
		alpha = math.pi/180*self.alpha 	# from deg to rad
		R = self.base_side*(3**0.5/6)  	# calculating radius of the base 
		r = self.EE_radius*(3**0.5/6)	# calculating radius of the EE
		a = self.active_rod 			# active rod length
		b = self.passive_rod			# passive rod length

		# assigning position of the EE 
		px = self.EE_position_global[0] # global position of EE - x direction
		py = self.EE_position_global[1] # global position of EE - y direction
		pz = self.EE_position_global[2] # global position of EE - z direction

		# initializing theta 1, 2, 3
		theta_1 = np.zeros((3))
		theta_2 = np.zeros((3))
		theta_3 = np.zeros((3))

		# calculating theta 1, 2, 3
		for i in [0, 1, 2]:
			theta_3[i] = math.acos((px*sin(alpha[i]) + py*cos(alpha[i]))/b) # theta_3 is 0independent of theta_1 and theta_2 

			A = px*cos(alpha[i]) - py*sin(alpha[i]) - R + r
			B = pz
			M = (A**2 + B**2 + a**2 - (b*sin(theta_3[i]))**2)/(2*a)
			t = (B + (B**2 - M**2 + A**2)**0.5)/(M + A)

			theta_1[i] = 2*math.atan(t) 
			theta_2[i] = math.asin((pz - a*sin(theta_1[i]))/(b*sin(theta_3[i]))) - theta_1[i]
		
		# now we have all of the theta 1, 2 and 3 
		self.theta_1 = theta_1
		self.theta_2 = theta_2
		self.theta_3 = theta_3

		return (theta_1, theta_2, theta_3)
	
	def get_jacobian_matrix(self): 
		# here we calculate jacobian matrix using the theta_1, theta_2 
		# and theta_3 
		
		# initializing J_ij 
		jx = np.zeros((3))
		jy = np.zeros((3))
		jz = np.zeros((3))
		J_P = np.zeros((3, 3))
		J_theta = np.zeros((3, 3))

		for i in [0, 1, 2]:
			jx[i] =  sin(self.theta_3[i])*cos(self.theta_2[i] + self.theta_1[i])*cos(self.alpha[i]) + cos(self.theta_3[i])*sin(self.alpha[i])
			jy[i] = -sin(self.theta_3[i])*cos(self.theta_2[i] + self.theta_1[i])*sin(self.alpha[i]) + cos(self.theta_3[i])*cos(self.alpha[i])
			jz[i] =  sin(self.theta_3[i])*sin(self.theta_2[i] + self.theta_1[i])
			J_P[i, :] = [jx[i], jy[i], jz[i]]
			J_theta[i, i] = sin(self.theta_2[i])*sin(self.theta_3[i])
		
		return (J_P, J_theta)


# =================================================================================================
# -- STEP 1: GENERATING TRAJECTORY ----------------------------------------------------------------
# =================================================================================================

# we make the path of circular motion of the EE as following equations: 
# x = R cos(t^3 - 1.5T*t^2)
# y = R sin(t^3 - 1.5T*t^2)
# z = constant


def circle_generator


# =================================================================================================
# -- STEP 2: GENERATING TRAJECTORY ----------------------------------------------------------------
# =================================================================================================





# =================================================================================================
# -- STEP 1: GENERATING TRAJECTORY ----------------------------------------------------------------
# =================================================================================================


# =================================================================================================
# -- MAIN -----------------------------------------------------------------------------------------
# =================================================================================================

circle_generator(1)