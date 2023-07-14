import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import numpy as np 

from delta_robot import DeltaRobot 
from delta_robot import tand, sind, cosd
from path_planning_mltp import PathPlannerMLTP

X = [0.05, 0.06, 0.05, 0, 0.04, 0.05]
Y = [0.05, -0.16, 0.06, -0.15, 0.05, 0.05]
Z = [-0.31, -0.35, -0.31, -0.42, -0.34, -0.31]



def main():

	# defining robot 
	delta_robot = DeltaRobot(0.2, 0.46, 0.1, 0.074)

	# path critera = [[x0, x1, ...xn], [y0, y1, ..., yn], [z0, z1, ..., zn]]
	path_criteria = [X, Y, Z]

	# defining path planner
	path_planner = PathPlannerMLTP(delta_robot, path_criteria, 1)

	# using 4567 planner
	results = path_planner.cubic_spline()
	_, _, _, _, _, ee_pos_t = results

	# DRAW INITIAL AND FINAL STATE
	fig = plt.figure(figsize=(4,4))

	ax = fig.add_subplot(111, projection='3d')

	for x, y, z in zip(X, Y, Z):
		ax.scatter(x, y, z, color='blue') 

	# l = x2 - x1
	# m = y2 - y1 
	# n = z2 - z1 
	# x = np.linspace(x1, x2, 100)
	# y = (x-x1)*m/l + y1
	# z = (x-x1)*n/l + z1

	# ax.plot(x, y, z, 'r--')


	# ANIMATION

	def update(num, data, line):
	    line.set_data(data[:2, :num])
	    line.set_3d_properties(data[2, :num])

	ee_pos_t = np.transpose(ee_pos_t)
	N = ee_pos_t.shape[1]

	line, = ax.plot(ee_pos_t[0], ee_pos_t[1], ee_pos_t[2], 'b')

	ani = animation.FuncAnimation(fig, update, N, fargs=(ee_pos_t, line), interval=10000/N, blit=False)

	ani.save('animation.gif', fps=240)



# run main 
if __name__ == "__main__":
	main()



# TASKS: 

	# calc the line in 3D space 
	# draw it in python (dashed)
