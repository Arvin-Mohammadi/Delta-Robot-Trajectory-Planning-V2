
# =================================================================================================
# -- imports --------------------------------------------------------------------------------------
# =================================================================================================

from delta_robot import DeltaRobot 
from delta_robot import tand, sind, cosd

import numpy as np 
import math 
import matplotlib.pyplot as plt 
from scipy.optimize import fsolve


# =================================================================================================
# -- point to point path planning class -----------------------------------------------------------
# =================================================================================================

class PathPlannerPTP: 
	def __init__(self, robot, ee_pos_i, ee_pos_f, thetadot_max):
		self.ee_pos_i 	= np.array(ee_pos_i)
		self.ee_pos_f 	= np.array(ee_pos_f)
		self.thetadot_max 	= thetadot_max*6 # convert rpm to deg/s
		self.theta_i = robot.inverse_kin(self.ee_pos_i).reshape((3, 1)) 
		self.theta_f = robot.inverse_kin(self.ee_pos_f).reshape((3, 1))

	def point_to_point_467(self):
		FREQUENCY = 1000 
		# overall time period
		T = 35/16*(self.theta_f - self.theta_i)/self.thetadot_max
		T = math.floor(max(T)*FREQUENCY)
		tau = np.array(range(0, T))/T

		# theta time profile
		s_tau = -20*tau**7 + 70*tau**6 - 84*tau**5 + 35*tau**4
		theta_t = np.array(self.theta_i) + np.array(self.theta_f - self.theta_i)*s_tau

		# theta dot time profile
		s_tau_d = -140*tau**6 + 420*tau**5 - 420*tau**4 + 140*tau**3
		theta_dot_t = np.array(self.theta_f - self.theta_i)/T*s_tau_d

		# checking the forward kinematics 
		ee_pos_t = np.zeros(theta_t.shape)
		for idx, i in enumerate(theta_t.transpose()):
			ee_pos_t[:, idx] = robot.forward_kin(theta_t[:, idx])

		# plot theta_t 
		plt.grid(True)
		plt.plot(tau, theta_t.transpose(), label=['theta_1', 'theta_2', 'theta_3'])
		plt.title("angle-time plot")
		plt.legend()
		plt.xlabel("normalized time")
		plt.ylabel("angle theta (deg)")
		plt.savefig("4567-theta.png")
		plt.clf()

		# plot theta_dot_t
		plt.grid(True)
		plt.plot(tau, theta_dot_t.transpose(), label=['theta_dot_1', 'theta_dot_2', 'theta_dot_3'])
		plt.title("angular velocity-time plot")
		plt.legend()
		plt.xlabel("normalized time")
		plt.ylabel("angular velocity theta_dot (deg/s)")
		plt.savefig("4567-theta-dot.png")
		plt.clf()

		# plot EE-position 
		plt.grid(True)
		plt.plot(tau, ee_pos_t.transpose(), label=['x', 'y', 'z'])
		plt.title("position-time plot")
		plt.legend()
		plt.xlabel("normalized time")
		plt.ylabel("EE position (m)")
		plt.savefig("4567-ee-position.png")
		plt.clf()


	def point_to_point_467_vivf(self, thetadot_i, thetadot_f):
		FREQUENCY = 1000 

		for idx in range(3):
			T = self._T_4567_vivf(self.theta_i[idx], self.theta_f[idx], thetadot_i, thetadot_f)

	
	def _T_4567_vivf(self, theta_i, theta_f, thetadot_i, thetadot_f):

		def equation1(variables):

			(tau, T) = variables

			a = -20 + 10*T/(theta_f - theta_i)*(thetadot_f + thetadot_i)
			b = 70 - 3*T/(theta_f - theta_i)*(17*thetadot_f + 18*thetadot_i)
			c = -56 + 3*T/(theta_f - theta_i)*(13*thetadot_f + 15*thetadot_i)
			d = 35 - 5*T/(theta_f - theta_i)*(3*thetadot_f + 4*thetadot_i)
			g = T*thetadot_i/(theta_f - theta_i)

			eqn1 = 42*a*tau**3 + 30*b*tau**2 + 20*c*tau + 12*d
			eqn2 = T - (7*a*tau**6 + 6*b*tau**5 + 5*c*tau**4 + 4*d*tau**3 + g)*(theta_f - theta_i)/self.thetadot_max 

			return np.reshape([eqn1, eqn2], (-1)) 


		# T_solution = fsolve(equation1, (0.5, 0))

		# print(T_solution)

		tau = np.linspace(0, 1, 100)
		T_guesses = np.linspace(0.0001, 2, 100) # modify the initial guess range here
		T_solution = None

		for T_guess in T_guesses:
			solution = fsolve(equation1, (0.5, T_guess))
			if solution[1] > 0.1: # check if T is positive and non-zero
				T_solution = solution[1]
				break

		if T_solution is not None:
			print("Minimum positive non-zero T value:", T_solution)
		else:
			print("No positive non-zero T solution found in the given range.")

# =================================================================================================
# -- main -------------------------------------------------------------------------------------
# =================================================================================================

if __name__ == "__main__":
	delta_robot = DeltaRobot(0.2, 0.46, 0.1, 0.074)
	path_planner = PathPlannerPTP(delta_robot, [0.05, 0.05, -0.31], [0, -0.15, -0.42], 20)
	path_planner.point_to_point_467_vivf(0, 0)