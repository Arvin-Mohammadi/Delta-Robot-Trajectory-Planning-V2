
# =================================================================================================
# -- imports --------------------------------------------------------------------------------------
# =================================================================================================

from delta_robot import DeltaRobot 
from delta_robot import tand, sind, cosd

import numpy as np 
import math 
import matplotlib.pyplot as plt 

# =================================================================================================
# -- point to point path planning class -----------------------------------------------------------
# =================================================================================================

class PathPlannerPTP: 
	def __init__(self, ee_pos_i, ee_pos_f, theta_dot_max):
		self.ee_pos_i 	= np.array(ee_pos_i)
		self.ee_pos_f 	= np.array(ee_pos_f)
		self.theta_i 	= np.zeros((3, 1))
		self.theta_f 	= np.zeros((3, 1))
		self.theta_dot_max 	= theta_dot_max*6 # convert rpm to deg/s

	def point_to_point_467(self, robot):
		self.theta_i = robot.inverse_kin(self.ee_pos_i).reshape((3, 1)) 
		self.theta_f = robot.inverse_kin(self.ee_pos_f).reshape((3, 1))

		FREQUENCY = 1000 
		# overall time period
		T = 35/16*(self.theta_f - self.theta_i)/self.theta_dot_max
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






# =================================================================================================
# -- main -------------------------------------------------------------------------------------
# =================================================================================================

if __name__ == "__main__":
	delta_robot = DeltaRobot(0.2, 0.46, 0.1, 0.074)
	path_planner = PathPlannerPTP([0.05, 0.05, -0.31], [0, -0.15, -0.42], 4050)
	path_planner.point_to_point_467(delta_robot)