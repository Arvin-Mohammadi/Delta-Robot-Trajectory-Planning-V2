import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import numpy as np 

from delta_robot import DeltaRobot 
from delta_robot import tand, sind, cosd
from path_planning_ptp import PathPlannerPTP


POINT1 = [0.05, 0.05, -0.31]
POINT2 = [0, -0.15, -0.42]



def main():

	# defining robot 
	delta_robot = DeltaRobot(0.2, 0.46, 0.1, 0.074)

	# defining path planner
	path_planner = PathPlannerPTP(delta_robot, POINT1, POINT2, 20)

	# using 4567 planner
	results_4567 = path_planner.point_to_point_4567()
	_, _, _, _, _, ee_pos_t = results_4567

	# DRAW INITIAL AND FINAL STATE
	fig = plt.figure(figsize=(4,4))

	ax = fig.add_subplot(111, projection='3d')

	[x1, y1, z1] = POINT1
	[x2, y2, z2] = POINT2
	ax.scatter(x1, y1, z1, color='blue') 
	ax.scatter(x2, y2, z2, color='blue') 

	l = x2 - x1
	m = y2 - y1 
	n = z2 - z1 
	x = np.linspace(x1, x2, 100)
	y = (x-x1)*m/l + y1
	z = (x-x1)*n/l + z1

	ax.plot(x, y, z, 'r--')


	# ANIMATION

	def update(num, data, line):
	    line.set_data(data[:2, :num])
	    line.set_3d_properties(data[2, :num])

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
