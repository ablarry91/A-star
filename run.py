'''
ME-449 Robotic Manipulation final homework submission
Coded by Austin Lawrence
Submitted to Professor Lynch
'''

'''1. Implement an A* path planner. The planner takes a graph G as input, with N nodes and E edges. Edge ei is just a specification of the two nodes (i,j) it connects as well as the distance between the two nodes, dij. One possible representation of G is as a symmetric matrix, where Gij = Gji = 0 if there is no edge between i and j, and Gij = Gji = dij if there is an edge. Your A* planner should return the sequence of nodes visited, {n1,n2,...,nk}, in the shortest path, where n1 is the start node and nk is the goal node. If there is no solution, your code should indicate so. Turn in your well-commented code.'''

'''2. Now test your A* code on the case of a circular mobile robot moving among N circular obstacles in a 100 x 100 planar region.
Input:
(i) the radius r of the robot
(ii) a list of radii ri and (xi,yi) coordinates of the centers of the obstacles
(iii) a list of (x,y) coordinates (the configuration of the center of the robot) for potential nodes of the graph
(iv) the start and goal nodes
Your code should then discard potential nodes that are in collision, construct the graph consisting of nodes that can be connected by straight-line paths without hitting an obstacle (you should come up with an exact method to determine whether two nodes are connected by a straight line, no sampling), invoke your A* planner, and give the result as a list of nodes in the shortest path as well as a graphical representation of the solution, as in Figure 1.
You may find it helpful, for testing your code, to create another program that invokes this program using a random set of obstacles and a random set of nodes.
Turn in your well-commented code, as well as the graphical output for one example, of similar complexity to what you see in Figure 1. You are also welcome to submit a more complicated example, for example generated by a random set of nodes and obstacles.
'''

import numpy as np
import matplotlib.pyplot as plt

def createTargets(size):
	import random
	targets = np.zeros([2,2])
	for i in range(2):
		targets[i,0] = random.uniform(size[0],size[1])
		targets[i,1] = random.uniform(size[2],size[3])
	# print 'my targets are','\n',targets
	return targets

def createObstacles(size, targets, quantity):
	import random
	radii = np.array([10,20]) #the range of possible radii for the obstacles
	obstacles = np.zeros([quantity, 3]) #Nx3 matrix, columns are X,Y, and radius

	#creates a random set of obstacles of random size
	for i in range(quantity):
		check = False #used for verifying if a node intercepts the targets
		while check == False:
			#try to create a node that does not overlap the targets
			nodeX = random.uniform(size[0],size[1])
			nodeY = random.uniform(size[2],size[3])
			nodeRadius = random.uniform(radii[0],radii[1])
			check = checkIntersection(targets, nodeX, nodeY, nodeRadius)
		obstacles[i,0] = nodeX
		obstacles[i,1] = nodeY
		obstacles[i,2] = nodeRadius
	return obstacles

def checkIntersection(targets, nodeX, nodeY, nodeRadius):
	#checks to see if the prospective nodes intersect with the start or finish targets
	for i in range(len(targets)):
		distance = np.sqrt(np.power(targets[i,0]-nodeX,2)+np.power(targets[i,1]-nodeY,2))
		if distance < nodeRadius:
			# print 'got an intersection at', nodeX, nodeY, nodeRadius
			return False
	return True

def createNodes(size, obstacles, quantity):
	#creates a random set of nodes
	import random
	nodes = np.zeros([quantity, 2]) #columns are X and Y
	for i in range(quantity):
		check = False
		while check == False:
			#try to create a node that does not overlap the obstacles
			nodeX = random.uniform(size[0],size[1])
			nodeY = random.uniform(size[2],size[3])
			#checks for interception on each individual obstacle
			for j in range(len(obstacles)):
				targets = np.matrix([nodeX, nodeY])
				check = checkIntersection(targets, obstacles[j,0], obstacles[j,1], obstacles[j,2])
				if check:
					nodes[i,0] = nodeX
					nodes[i,1] = nodeY
					break
	return nodes

def createEdges(nodes, obstacles, targets):
	pass

def aStar():
	pass

size = np.array([0,100,0,100]) #xMin, xMax, yMin, yMax
targets = createTargets(size) #xStart, yStart, xGoal, yGoal
obstacles = createObstacles(size, targets, 10) #xNode, yNode, radius
nodes = createNodes(size, obstacles, 50)
edges = createEdges(nodes, obstacles, targets)
aStar()

# plt.scatter(nodes[:,0], nodes[:,1], s=1, alpha=0.5)
# # plt.scatter(targets[:,0], targets[:,1], s=500, c='red',alpha=0.5)
# plt.scatter(50, 50, s=10000, c='red',alpha=0.5)
# plt.scatter(obstacles[:,0], obstacles[:,1], s=obstacles[2], c='green',alpha=0.5)
# plt.Circle((50,50),10,color='r')

# plt.show()

print obstacles,'\n',nodes


fig = plt.figure()
ax = fig.add_subplot(1,1,1)
for i in range(len(obstacles)):
	circ = plt.Circle((obstacles[i,0],obstacles[i,1]), radius = obstacles[i,2],alpha=0.8, color='blue')
	ax.add_patch(circ)
for i in range(len(targets)):
	circ = plt.Circle((targets[i,0],targets[i,1]), radius = 3, color='red')
	ax.add_patch(circ)
for i in range(len(nodes)):
	circ = plt.Circle((nodes[i,0],nodes[i,1]), radius = 1, color='black')
	ax.add_patch(circ)
plt.axis(size, aspect=1)	
plt.show()






