import numpy as np 
import matplotlib.pyplot as plt 

from delta_robot import DeltaRobot

def main():

	X = np.arange(start=-1, stop=1, step=0.1)
	Y= np.arange(start=-1, stop=1, step=0.1)
	Z = np.arange(start=-1, stop=1, step=0.1)

	delta = DeltaRobot(0.2, 0.46, 0.1, 0.074)

		# DRAW INITIAL AND FINAL STATE
	fig = plt.figure(figsize=(4,4))

	ax = fig.add_subplot(111, projection='3d')

	for x in X:
		for y in Y: 
			for z in Z: 
				if isinstance(delta.inverse_kin([x, y, z]),int):
					pass 
				else:
					ax.scatter(x, y, z, color='blue') 

	plt.savefig("reachable_workspace.png")


if __name__=="__main__":
	main()


