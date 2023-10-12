
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
# -- dirty calculations ---------------------------------------------------------------------------
# =================================================================================================
class Coeff:

	def __init__(self, points):

		# initialzing the postion, velocity and acceleration vectors 
		# all of the vectors are n*3 matrices (e.g: for v we have 3 components in x, y and z direction)
		self.points = np.array(points)
		self.t = np.zeros(self.points.shape)
		self.n = self.points.shape[0]

		# time value assignment, shape = n*3
		for i in [0, 1, 2]:
			self.t[:, i] = np.linspace(0, self.n, num=self.n)
		
		# time intervals (T), shape = n-1*3
		self.T = self.t[1:self.n, :] - self.t[0:self.n-1, :]

	def velocity(self, initial_velo=[0, 0, 0], final_velo=[0, 0, 0]):
		# initializing c_prime and A_prime matrices (last dimension is for the x, y, z directions)
		A_prime = np.zeros((3, self.n-2, self.n-2))
		c_prime = np.zeros((self.n-2, 3))
		velocity_profile = np.zeros(self.points.shape)
		T = self.T
		
		# makin the A_prime and c_prime matrices 
		for i in range(A_prime.shape[1]):
			if i != 0:
				A_prime[:, i, i-1] = T[i+1]
			A_prime[:, i, i] = 2*(T[i] + T[i+1])
			if i != A_prime.shape[1]-1:
				A_prime[:, i, i+1] = T[i]

		for i in range(A_prime.shape[1]):
			c_prime[i, :] = 3/(T[i]*T[i+1])*(T[i]**2*(self.points[i+2, :] - self.points[i+1, :]) + T[i+1]**2*(self.points[i+1, :] - self.points[i, :]))
			if i == 0:
				c_prime[i, :] -= T[i+1]*initial_velo
			elif i == A_prime.shape[1]-1:
				c_prime[i, :] -= T[i]*final_velo

		# calculating v vector from A_prime and C_prime matrices 
		v = np.zeros((self.n-2, 3))
		for i in [0, 1, 2]:
			M = np.linalg.inv(A_prime[i, :, :])
			N = c_prime[:, i]
			v[:, i] = np.matmul(M, N)

		velocity_profile[0, :] = initial_velo
		velocity_profile[self.n-1, :] = final_velo
		velocity_profile[1:self.n-1, :] = v

		return velocity_profile

	def coeff_matrix(self, velocity_profile):
		# initializing the coefficient matrix 
		# dim 1 == number of polynomials 				--> k = 0, ..., n-2
		# dim 2 == number of x, y, z directions 		--> 3
		# dim 3 == number of coeff in the polynomial 	--> (e.g: a[k][0][m] that m=0,1,2,3 is for the k_th point in x direction)
		coeff = np.zeros((self.n-1, 3, 4)) 
		
		# assigning the values of wrt position and velocity
		coeff[:, :, 0] = self.points[0:self.n - 1, :]
		coeff[:, :, 1] = velocity_profile[0:self.n - 1, :] 
		coeff[:, :, 2] = 1/self.T*( 3*(self.points[1:self.n, :] - self.points[0:self.n-1, :])/self.T - 2*velocity_profile[0:self.n-1, :] - velocity_profile[1:self.n, :])
		coeff[:, :, 3] = 1/self.T**2*( 2*(- self.points[1:self.n, :] + self.points[0:self.n-1, :])/self.T + velocity_profile[0:self.n-1, :] + velocity_profile[1:self.n, :])
		
		return coeff

# =================================================================================================
# -- point to point path planning class -----------------------------------------------------------
# =================================================================================================

class PathPlannerMLTP: 
	def __init__(self, robot, path_criteria, max_velo):
		self.robot = robot

		if not isinstance(path_criteria, int): 
			self.path_criteria   = np.transpose(np.array(path_criteria))
			self.path_criteria_x = np.array(path_criteria[0])
			self.path_criteria_y = np.array(path_criteria[1])
			self.path_criteria_z = np.array(path_criteria[2])
			self.max_velo = max_velo
			self.n = self.path_criteria_x.shape[0] - 1

	def cubic_spline(self):
		FREQUENCY = 10000

		# find n
		n = self.n

		# find overall T_total
		d = 0 # rough distance
		for i in range(1, n+1):
			d += ((self.path_criteria_x[i] - self.path_criteria_x[i-1])**2 + (self.path_criteria_y[i] - self.path_criteria_y[i-1])**2 + (self.path_criteria_z[i] - self.path_criteria_z[i-1])**2)**0.5
		d = d*1.5

		# T_total = d/self.max_velo

		# comment if don't want normalized 
		T_total = 1 
		self.max_velo = d/T_total

		# calculating IK for path criteria
		theta = np.zeros(self.path_criteria.shape)

		for idx, i in enumerate(self.path_criteria):
			theta[idx] = self.robot.inverse_kin(i)

		# find the coeff matrix
		coeff = Coeff(theta) 	# initializing the coeff class (for interpolating theta profile)
		velocity_profile = coeff.velocity() 	# calculating velocity profile
		coeff_matrix = coeff.coeff_matrix(velocity_profile) 	# calculating coefficient matrix of a_ij

		t =  coeff.t.transpose()[0]


		# getting the outputs (inputs of the stepper motor)
		t_output = np.arange(0, math.ceil(T_total*FREQUENCY))

		# build T vector for using on coeff matrix 
		T_output = np.arange(0, math.ceil(T_total*FREQUENCY))/math.ceil(T_total*FREQUENCY)*t[-1]
		T_old = np.copy(T_output) # not important 
		counter = 1
		for idx, i in enumerate(T_output):
			try:
				if i>= t[counter+1]:
					counter += 1
			except: 
				pass 
			if i >= t[counter]:
				T_output[idx] = T_output[idx] - t[counter]

		T_output = np.transpose(np.array([T_output, T_output, T_output]))

		# get final profile outputs of position, velocity, acceleration, jerk
		theta_output 			= np.zeros((t_output.shape[0], 3))
		thetadot_output 		= np.zeros((t_output.shape[0], 3))
		thetadotdot_output 		= np.zeros((t_output.shape[0], 3))
		thetadotdotdot_output   = np.zeros((t_output.shape[0], 3))

		counter = 0 
		# print(T_old)
		for i in t_output:
			try:
				if T_old[i] >= t[counter+1]:
					counter += 1
			except: 
				pass 

			theta_output[i] 			= 	np.transpose(coeff_matrix[counter])[0] + \
											np.transpose(coeff_matrix[counter])[1]*T_output[i] + \
											np.transpose(coeff_matrix[counter])[2]*T_output[i]**2 + \
											np.transpose(coeff_matrix[counter])[3]*T_output[i]**3
			
			thetadot_output[i] 			= 	np.transpose(coeff_matrix[counter])[1] + \
											np.transpose(coeff_matrix[counter])[2]*T_output[i]*2 + \
											np.transpose(coeff_matrix[counter])[3]*T_output[i]**2*3
			
			thetadotdot_output[i] 		= 	np.transpose(coeff_matrix[counter])[2]*2 + \
											np.transpose(coeff_matrix[counter])[3]*T_output[i]*6
			
			thetadotdotdot_output[i] 	= 	np.transpose(coeff_matrix[counter])[3]*6


		ee_pos_output = np.zeros(theta_output.shape)
		# print(theta_output.shape)
		for idx, i in enumerate(theta_output):
			ee_pos_output[idx] = self.robot.forward_kin(theta_output[idx])



		t_output = np.transpose(np.array([t_output, t_output, t_output]))

		return (t_output, theta_output, thetadot_output, thetadotdot_output, thetadotdotdot_output, ee_pos_output)


	def trapezoidal_mltp(self, acceleration):
		FREQUENCY = 1000

		# calculate each time period from the max velocity 

		for i in range(self.n):
			prev_point  = [self.path_criteria_x[i], self.path_criteria_y[i], self.path_criteria_z[i]]
			next_point 	= [self.path_criteria_x[i+1], self.path_criteria_y[i+1], self.path_criteria_z[i+1]]

		
	def _triangular_ptp(self, point1, point2): 
		pass 

	def higher_order_poly_3pt(self, q0, q1, q2): # thi will only work if you have 3 points to interpolate 
		
		FREQUENCY = 100

		q0 = self.robot.inverse_kin(np.array(q0))
		q1 = self.robot.inverse_kin(np.array(q1))
		q2 = self.robot.inverse_kin(np.array(q2))

		a0 = q0
		a1 = np.array([0, 0, 0])
		a2 = np.array([0, 0, 0])
		a3 = np.array([0, 0, 0])
		a4 = 256*q1 - 163*q0 - 93*q2
		a5 = 596*q0 - 1024*q1 + 428*q2
		a6 = 1536*q1 - 838*q0 - 698*q2
		a7 = 532*q0 - 1024*q1 + 492*q2
		a8 = 256*q1 - 128*q0 - 128*q2

		t = np.array(range(FREQUENCY))/FREQUENCY
		q_t = np.zeros((FREQUENCY, 3))
		qdot_t = np.zeros((FREQUENCY, 3))
		qddot_t = np.zeros((FREQUENCY, 3))
		qdddot_t = np.zeros((FREQUENCY, 3))


		for i in range(FREQUENCY):
			q_t[i] = a8*t[i]**8 + a7*t[i]**7 + a6*t[i]**6 + a5*t[i]**5 + a4*t[i]**4 + a3*t[i]**3 + a2*t[i]**2 + a1*t[i] + a0
			qdot_t[i] = 8*a8*t[i]**7 + 7*a7*t[i]**6 + 6*a6*t[i]**5 + 5*a5*t[i]**4 + 4*a4*t[i]**3 + 3*a3*t[i]**2 + 2*a2*t[i] + a1 
			qddot_t[i] = 56*a8*t[i]**6 + 42*a7*t[i]**5 + 30*a6*t[i]**4 + 20*a5*t[i]**3 + 12*a4*t[i]**2 + 6*a3*t[i] + 2*a2
			qdddot_t[i] = 336*a8*t[i]**5 + 210*a7*t[i]**4 + 120*a6*t[i]**3 + 60*a5*t[i]**2 + 24*a4*t[i] + 6*a3

		# temp_t = 1
		# temp  = a8*temp_t**8 + a7*temp_t**7 + a6*temp_t**6 + a5*temp_t**5 + a4*temp_t**4 + a3*temp_t**3 + a2*temp_t**2 + a1*temp_t + a0
		# print(temp)

		ee_pos = np.zeros((FREQUENCY, 3))
		for idx, i in enumerate(np.array(q_t)):
			ee_pos[idx] = self.robot.forward_kin(q_t[idx])


		results = (t, q_t, qdot_t, qddot_t, qdddot_t, ee_pos)

		return results

	def plot(self, results):
		(t_output, theta_output, thetadot_output, thetadotdot_output, thetadotdotdot_output, ee_pos_output) = results


		fig = plt.figure()
		fig.set_figheight(15)
		fig.set_figwidth(10)

		# plot theta_t 
		plt.subplot(511)
		plt.grid(True)
		plt.plot(t_output, theta_output, label=['theta_1', 'theta_2', 'theta_3'], linewidth=10)
		plt.title("angle-time plot", fontsize=20)
		plt.legend()
		plt.xlabel("time (ms)", fontsize=15)
		plt.ylabel("angle (deg)", fontsize=15)

		# plot theta_dot_t
		plt.subplot(512)
		plt.grid(True)
		plt.plot(t_output, thetadot_output, label=['theta_dot_1', 'theta_dot_2', 'theta_dot_3'], linewidth=10)
		plt.title("angular velocity-time plot", fontsize=20)
		plt.legend()
		plt.xlabel("time (ms)", fontsize=15)
		plt.ylabel("angular velocity (deg/s)", fontsize=15)

		# plot theta_ddot_t 
		plt.subplot(513)
		plt.grid(True)
		plt.plot(t_output, thetadotdot_output, label=['x', 'y', 'z'], linewidth=10)
		plt.title("angular acceleration-time plot", fontsize=20)
		plt.legend()
		plt.xlabel("time (ms)", fontsize=15)
		plt.ylabel("angular acceleration (deg/s^2)", fontsize=15)

		# plot theta_dddot_t 
		plt.subplot(514)
		plt.grid(True)
		plt.plot(t_output, thetadotdotdot_output, label=['x', 'y', 'z'], linewidth=10)
		plt.title("angular jerk-time plot", fontsize=20)
		plt.legend()
		plt.xlabel("time (ms)", fontsize=15)
		plt.ylabel("angular jerk (deg/s^3)", fontsize=15)

		# plot EE-position 
		plt.subplot(515)
		plt.grid(True)
		plt.plot(t_output, ee_pos_output, label=['x', 'y', 'z'], linewidth=10)
		plt.title("position-time plot", fontsize=20)
		plt.legend()
		plt.xlabel("time (ms)", fontsize=15)
		plt.ylabel("EE position (m)", fontsize=15)


		plt.tight_layout()
		plt.savefig("cubic spline metho - 2.pdf", format="pdf")
		plt.clf()



# =================================================================================================
# -- paper results -------------------------------------------------------------------------------
# =================================================================================================


def paper_results():

	# defining the robot 
	delta_robot = DeltaRobot(30.9, 59.5, 13, 10)

	# path critera = [[x0, x1, ...xn], [y0, y1, ..., yn], [z0, z1, ..., zn]]
	# path_criteria = np.array([[-10, 10, -55], [10, -5, -70], [0, 0, -60]]).T
	path_criteria = np.array([[-10, 10, -55] ,  [10, -5, -70] ,  [0, 0, -60] ,  [5, -5, -70] ,  [2, -3, -60] ,  [12, 0, -65] ,  [-5, -10, -55] ,  [-10, 10, -55] ,  [10, -5, -70] ,  [0, 0, -60] ,  [5, -5, -70] ,  [2, -3, -60] ,  [12, 0, -65] ,  [-5, -10, -55] ,  [-10, 10, -55] ,  [10, -5, -70] ,  [0, 0, -60] ,  [5, -5, -70] ,  [2, -3, -60] ,  [12, 0, -65] ,  [-5, -10, -55] ]).T

	# defining the path planner 
	path_planner = PathPlannerMLTP(delta_robot, path_criteria, 1)

	# cubic spline 
	cubic_spline_results = path_planner.cubic_spline()
	path_planner.plot(cubic_spline_results)


	q0 = [-10, 10, -55]
	q1 = [10, -5, -70]
	q2 = [0, 0, -60]

	# higher order polynomial
	# results = path_planner.higher_order_poly_3pt(q0, q1, q2)
	# path_planner.plot(results)


# =================================================================================================
# -- main -----------------------------------------------------------------------------------------
# =================================================================================================

if __name__ == "__main__":

	paper_results()

	# TEST 1

	# # defining robot 
	# delta_robot = DeltaRobot(0.2, 0.46, 0.1, 0.074)

	# # path critera = [[x0, x1, ...xn], [y0, y1, ..., yn], [z0, z1, ..., zn]]
	# path_criteria = [[0.05, 0, 0.05, 0, 0.05, 0], [0.05, -0.15, 0.05, -0.15, 0.05, -0.15], [-0.31, -0.42, -0.31, -0.42, -0.31, -0.42]]

	# # defining path planner
	# path_planner = PathPlannerMLTP(delta_robot, path_criteria, 1)

	# # # cubic spline 
	# # cubic_spline_results = path_planner.cubic_spline()
	# # path_planner.plot(cubic_spline_results)

	# # trapezoidal method
	# path_planner.trapezoidal_mltp(0.2)



	# # TEST 2


	# # defining robot 
	# delta_robot = DeltaRobot(0.2, 0.46, 0.1, 0.074)

	# # path critera = [[x0, x1, ...xn], [y0, y1, ..., yn], [z0, z1, ..., zn]]
	# path_criteria = [[0, 0, 0, 0], [-0.05, -0.05, 0, 0], [-0.42, -0.31, -0.31, -0.42]]

	# # defining path planner
	# path_planner = PathPlannerMLTP(delta_robot, path_criteria, 1)

	# # # cubic spline 
	# cubic_spline_results = path_planner.cubic_spline()
	# path_planner.plot(cubic_spline_results)

	# (_, _, _, _, _, eepos) = cubic_spline_results

	# X = eepos.transpose()[0]
	# Y = eepos.transpose()[1]
	# Z = eepos.transpose()[2]

	# plt.plot(Y, Z)
	# plt.savefig("eeposition_cubicspline.png")


	# TEST 3 


	# # defining robot 
	# delta_robot = DeltaRobot(0.2, 0.46, 0.1, 0.074)

	# q0 = [0, -0.05, -0.42]
	# q1 = [0.05, 0, -0.31]
	# q2 = [0, 0.05, -0.42]

	# path_criteria = -1

	# path_planner = PathPlannerMLTP(delta_robot, path_criteria, 1)
	# results = path_planner.higher_order_poly_3pt(q0, q1, q2)
	# path_planner.plot(results)


	# TEST 4 

	# defining the robot 
	# delta_robot = DeltaRobot(30.9, 59.5, 13, 10)

	# # path critera = [[x0, x1, ...xn], [y0, y1, ..., yn], [z0, z1, ..., zn]]
	# path_criteria = np.array([[0, 0, -48.898], [0, 0, -55], [5, 5, -55], [0, 0, -48.898], [-5, 0, -60]]).T

	# # defining the path planner 
	# path_planner = PathPlannerMLTP(delta_robot, path_criteria, 0)

	# # cubic spline 
	# cubic_spline_results = path_planner.cubic_spline()
	# path_planner.plot(cubic_spline_results)

	# (_, ThetaVector, _, _, _, _) = cubic_spline_results
	# print(ThetaVector.shape)


