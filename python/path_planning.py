
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
		self.robot = robot


	def point_to_point_345(self):

		FREQUENCY = 1000 
		# overall time period
		T = 15/8*(self.theta_f - self.theta_i)/self.thetadot_max
		T = math.floor(max(T)*FREQUENCY)
		tau = np.array(range(0, T))/T

		# theta time profile
		s_tau = 6*tau**5 - 15*tau**4 + 10*tau**3
		theta_t = np.array(self.theta_i) + np.array(self.theta_f - self.theta_i)*s_tau

		# theta dot time profile
		s_tau_d = 30*tau**4 - 60*tau**3 + 30*tau**2
		theta_dot_t = np.array(self.theta_f - self.theta_i)/T*s_tau_d

		# theta double dot time profile
		s_tau_dd = 120*tau**3 - 180*tau**2 + 60*tau
		theta_ddot_t = np.array(self.theta_f - self.theta_i)/T*s_tau_dd

		# theta triple dot time profile
		s_tau_ddd = -360*tau**2 - 360*tau + 60
		theta_dddot_t = np.array(self.theta_f - self.theta_i)/T*s_tau_ddd

		# checking the forward kinematics 
		ee_pos_t = np.zeros(theta_t.shape)
		for idx, i in enumerate(theta_t.transpose()):
			ee_pos_t[:, idx] = self.robot.forward_kin(theta_t[:, idx])

		return (tau, theta_t, theta_dot_t, theta_ddot_t, theta_dddot_t, ee_pos_t)


	def point_to_point_4567(self):
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

		# theta double dot time profile
		s_tau_dd = -840*tau**5 + 2100*tau**4 - 1680*tau**3 + 420*tau**2
		theta_ddot_t = np.array(self.theta_f - self.theta_i)/T*s_tau_dd

		# theta triple dot time profile
		s_tau_ddd = -4200*tau**4 + 8400*tau**3 - 5040*tau**2 + 840*tau**1
		theta_dddot_t = np.array(self.theta_f - self.theta_i)/T*s_tau_ddd

		# checking the forward kinematics 
		ee_pos_t = np.zeros(theta_t.shape)
		for idx, i in enumerate(theta_t.transpose()):
			ee_pos_t[:, idx] = self.robot.forward_kin(theta_t[:, idx])

		return (tau, theta_t, theta_dot_t, theta_ddot_t, theta_dddot_t, ee_pos_t)


	def trapezoidal(self): 
		FREQUENCY = 1000

		# overall time period 

		# theta time profile 

		# theta dot time profile

		# theta double dot time profile

		# theta triple dot time profile

		# checking the forward kinematics 



	def plot_results(self, results):

		(tau, theta_t, theta_dot_t, theta_ddot_t, theta_dddot_t, ee_pos_t) = results

		fig = plt.figure()
		fig.set_figheight(15)
		fig.set_figwidth(10)

		# plot theta_t 
		plt.subplot(511)
		plt.grid(True)
		plt.plot(tau, theta_t.transpose(), label=['theta_1', 'theta_2', 'theta_3'], linewidth=10)
		plt.title("angle-time plot", fontsize=20)
		plt.legend()
		plt.xlabel("normalized time", fontsize=15)
		plt.ylabel("angle (deg)", fontsize=15)

		# plot theta_dot_t
		plt.subplot(512)
		plt.grid(True)
		plt.plot(tau, theta_dot_t.transpose(), label=['theta_dot_1', 'theta_dot_2', 'theta_dot_3'], linewidth=10)
		plt.title("angular velocity-time plot", fontsize=20)
		plt.legend()
		plt.xlabel("normalized time", fontsize=15)
		plt.ylabel("angular velocity (deg/s)", fontsize=15)

		# plot theta_ddot_t 
		plt.subplot(513)
		plt.grid(True)
		plt.plot(tau, theta_ddot_t.transpose(), label=['x', 'y', 'z'], linewidth=10)
		plt.title("angular acceleration-time plot", fontsize=20)
		plt.legend()
		plt.xlabel("normalized time", fontsize=15)
		plt.ylabel("angular acceleration (deg/s^2)", fontsize=15)

		# plot theta_dddot_t 
		plt.subplot(514)
		plt.grid(True)
		plt.plot(tau, theta_dddot_t.transpose(), label=['x', 'y', 'z'], linewidth=10)
		plt.title("angular jerk-time plot", fontsize=20)
		plt.legend()
		plt.xlabel("normalized time", fontsize=15)
		plt.ylabel("angular jerk (deg/s^3)", fontsize=15)

		# plot EE-position 
		plt.subplot(515)
		plt.grid(True)
		plt.plot(tau, ee_pos_t.transpose(), label=['x', 'y', 'z'], linewidth=10)
		plt.title("position-time plot", fontsize=20)
		plt.legend()
		plt.xlabel("normalized time", fontsize=15)
		plt.ylabel("EE position (m)", fontsize=15)


		plt.tight_layout()
		plt.savefig("4567 method.png")
		plt.clf()


# =================================================================================================
# -- main -------------------------------------------------------------------------------------
# =================================================================================================

if __name__ == "__main__":

	# defining robot 
	delta_robot = DeltaRobot(0.2, 0.46, 0.1, 0.074)

	# defining path planner
	path_planner = PathPlannerPTP(delta_robot, [0.05, 0.05, -0.31], [0, -0.15, -0.42], 20)

	# using 345 planner 
	results_345 = path_planner.point_to_point_345()
	path_planner.plot_results(results_345)

	# using 4567 planner
	results_4567 = path_planner.point_to_point_4567()
	path_planner.plot_results(results_4567)
