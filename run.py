'''
ME-449 Robotic Manipulation final homework submission
Coded by Austin Lawrence
Submitted to Professor Lynch
'''

import numpy as np
import matplotlib.pyplot as plt
import time

#creates a start and finish node in random locations
def createTargets(size):
	import random
	targets = np.zeros([2,2])
	for i in range(2):
		targets[i,0] = random.uniform(size[0],size[1])
		targets[i,1] = random.uniform(size[2],size[3])
	# print 'my targets are','\n',targets
	return targets

#creates inpenetrable obstacles that do not intersect any placed nodes
def createObstacles(size, targets, quantity):
	import random
	radii = np.array([5,10]) #the range of possible radii for the obstacles
	obstacles = np.zeros([quantity, 3]) #Nx3 matrix, columns are X,Y, and radius

	#creates a random set of obstacles of random size
	for i in range(quantity):
		check = False #used for verifying if a node intercepts the targets
		while check == False:
			#try to create a node that does not overlap the targets
			nodeX = random.uniform(size[0],size[1])
			nodeY = random.uniform(size[2],size[3])
			nodeRadius = random.uniform(radii[0],radii[1])
			check = checkIntersection(targets, nodeX, nodeY, nodeRadius+radius)
		obstacles[i,0] = nodeX
		obstacles[i,1] = nodeY
		obstacles[i,2] = nodeRadius
	return obstacles

#checks to see if a coordinate intersects an obstacle
def checkIntersection(targets, nodeX, nodeY, nodeRadius):
	#checks to see if the prospective nodes intersect with the start or finish targets
	for i in range(len(targets)):
		distance = np.sqrt(np.power(targets[i,0]-nodeX,2)+np.power(targets[i,1]-nodeY,2))
		if distance < nodeRadius:
			# print 'got an intersection at', nodeX, nodeY, nodeRadius
			return False
	return True

#creates a node that the robot could move to, not intersecting the obstacles already placed
def createNodes(size, obstacles, quantity):
	import random
	nodes = np.zeros([quantity, 2]) #columns are X and Y
	#try to place each node, checking for intersections
	for i in range(quantity):
		check = False
		while check == False:
			#try to create a node that does not overlap the obstacles
			nodeX = random.uniform(size[0],size[1])
			nodeY = random.uniform(size[2],size[3])
			#checks for interception on each individual obstacle
			for j in range(len(obstacles)):
				targets = np.matrix([nodeX, nodeY])
				test = checkIntersection(targets, obstacles[j,0], obstacles[j,1], obstacles[j,2]+radius)  #places node a body radius away from obstacles
				if test == False:
					break
			if test:
				check = True
				nodes[i,0] = nodeX
				nodes[i,1] = nodeY		
	return nodes

#connects the dots, so to speak.  Returns a dictionary that contains all the connections between nodes.  Checks for intersections.
def createEdges(nodes, obstacles, targets):
	import itertools
	import math
	from shapely.geometry import Point
	from shapely.geometry import MultiPoint

	nodes = np.vstack([nodes,targets])
	edgeDict = {}

	#builds your empty dictionary
	for i in nodes:
		edgeDict[hash(str(i))] = None

	#goes through every single combination of 2 nodes and evaluates if it intersects an obstacle.  
	for i in itertools.combinations(nodes,2):
		if i[1][0]-i[0][0] == 0:
			#it is vertical
			thetaInv = math.pi
			slope = float('inf')
			intercept = i[1][1] - slope*i[1][0]

		# slope = (i[1][1]-i[0][1])/(i[1][0]-i[0][0])
		# if slope == 0:
		# 	thetaInv = 0
		else:
			slope = round((i[1][1]-i[0][1])/(i[1][0]-i[0][0]),10)
			if slope == 0:
				thetaInv = 0
			else:
				thetaInv = math.atan(-1/slope)
			intercept = i[1][1] - slope*i[1][0]

		# intercept = i[1][1] - slope*i[1][0]
		arrayX = np.array([i[0][0],i[1][0]])
		arrayY = np.array([i[0][1],i[1][1]])

		test = True
		#iterates through every obstacle, builds a polygon around your two nodes, checks to see if the obstacle's center point falls within it.  If yes, then don't the path.
		for j in range(len(obstacles)):
			poly = np.zeros([4,2])
			dX = (obstacles[j,2]+radius)*np.cos(thetaInv)
			dY = (obstacles[j,2]+radius)*np.sin(thetaInv)

			poly[0,0] = i[0][0] + dX
			poly[0,1] = i[0][1] + dY
			poly[1,0] = i[0][0] - dX
			poly[1,1] = i[0][1] - dY
			poly[2,0] = i[1][0] + dX
			poly[2,1] = i[1][1] + dY
			poly[3,0] = i[1][0] - dX
			poly[3,1] = i[1][1] - dY	

			poly = MultiPoint(poly).convex_hull
			point = Point(obstacles[j,0],obstacles[j,1])

			if poly.contains(point):
				test = False
				break

		#if you successfully draw a polygon without intersections, add it to your dictionary as a useable edge
		if test:
			try:
				edgeDict[hash(str(i[0]))] = np.vstack([edgeDict[hash(str(i[0]))],i[1]])
			except:
				edgeDict[hash(str(i[0]))] = i[1]
			try:
				edgeDict[hash(str(i[1]))] = np.vstack([edgeDict[hash(str(i[1]))],i[0]])
			except:
				edgeDict[hash(str(i[1]))] = i[0]

	return edgeDict

#finds euclidian distance between two points
def evalHeuristic(current,goal):
	try:
		return np.sqrt(np.power(goal[1]-current[1],2)+np.power(goal[0]-current[0],2))
	except:
		print 'error in evalHeuristic.','\n',current,'\n',goal

#similar to evalHeuristic, but takes into account prior cost as well
def evalTrueCost(current, target, priorCost):
	return np.sqrt(np.power(target[1]-current[1],2)+np.power(target[0]-current[0],2)) + priorCost

#the big momma
def aStar(nodes, targets, edges):
	untouchedNodes = np.vstack([nodes,targets]) #nodes that we haven't expanded yet and may want to
	totalCostList = []  #tracks costs of nodes that we might want to expand
	trueCostList = []  #the actual distance traversed to get to a particular node
	expandedList = []  #tracks nodes that we might want to expand
	parentList = [] #a list of parent nodes so that we can build our path after the goal has been met
	touchedNodes = [] #a list of nodes that we've evaluated.  Same length as parentList

	start = targets[0]
	goal = targets[1]
	current = start

	touchedNodes.append(current)
	parentList.append(current)

	#expand your first node
	for newEdge in edges[hash(str(current))]:
		heuristic = evalHeuristic(newEdge, goal) 
		trueCost = evalTrueCost(current,newEdge,0) #finds distance traversed to find this node

		#update lists
		totalCost = heuristic + trueCost
		trueCostList.append(trueCost)
		totalCostList.append(totalCost)
		expandedList.append(newEdge)
		parentList.append(current)
		touchedNodes.append(newEdge)

	#pick the best (lowest cost) node to expand next
	lowestCostIndex = totalCostList.index(min(totalCostList))
	current = expandedList[lowestCostIndex]
	heuristic = evalHeuristic(current, goal)

	# print 'i choose,',current

	parentCost = trueCostList[lowestCostIndex]
	count = 0  #in case we're infinitely looping

	#keep doing this procedure until you find a solution or run out of options
	while heuristic != 0:
		#remove the node you just expanded
		expandedList.pop(lowestCostIndex)
		trueCostList.pop(lowestCostIndex)
		totalCostList.pop(lowestCostIndex)

		#expand your new node
		for newEdge in edges[hash(str(current))]:
			#check to see if we've expanded this node already
			skip = False
			for touched in touchedNodes:	
				try:
					if newEdge[0] == touched[0] and newEdge[1] == touched[1]:
						# print 'found a duplicate'
						skip = True
						break
				except:
					print "I think I'm out of choices here.  Giving up."
					skip = True
					# return False
					break
			if skip:
				continue

			#evaluate costs
			heuristic = evalHeuristic(newEdge, goal)
			trueCost = evalTrueCost(current,newEdge,parentCost)

			#update lists
			totalCost = heuristic + trueCost
			trueCostList.append(trueCost)
			totalCostList.append(totalCost)
			expandedList.append(newEdge)
			parentList.append(current)
			touchedNodes.append(newEdge)

		#pick the best node to expand
		try:
			lowestCostIndex = totalCostList.index(min(totalCostList))
		except:
			print 'No solution found.  I give up.'
			return []
			break
			#if this is empty, then we've exhausted our options.

		#update current node
		current = expandedList[lowestCostIndex]

		#update costs
		heuristic = evalHeuristic(current, goal)
		parentCost = trueCostList[lowestCostIndex]

		#in case you're indefinitely looping...
		count += 1
		if count >50:
			break

	#build your path from finish to start
	path = []
	index = lowestCostIndex
	count = 0

	#build the path from your goal node to your start node
	while round(current[0],6) != round(start[0],6) and round(current[1],6) != round(start[1],6):
		for i in range(len(touchedNodes)):
			if current[0] == touchedNodes[i][0] and current[1] == touchedNodes[i][1]:
				index = i
				break
		#look up your parent and set that as your new 'current' node
		parent = parentList[index]
		path.append(parent)
		current = parent
		count += 1
		if count > 20:
			break
	# print 'my path is','\n',path,'\n'
	return path

#get your initial time
start = time.time()

#build your grid, start/finish, obstacles, and nodes
radius = 0
size = np.array([0,100,0,100]) #xMin, xMax, yMin, yMax
targets = createTargets(size) #xStart, yStart, xGoal, yGoal
obstacles = createObstacles(size, targets, 40) #xNode, yNode, radius
nodes = createNodes(size, obstacles,40)

#set up plots everything
fig = plt.figure()
ax = fig.add_subplot(1,1,1)


# node locations:
 
# x          y
# 25       25
# 25       50
# 25       75  start
# 50       25
# 50       50
# 50       75  goal
# 75       25
# 75       50
# 75       75
 
# obstacle centers and radii:
 
# x          y          radius
# 30       30       4
# 37       62       6
# 37       75       5
# 50       62       3
# 62       50       4

#INPUT YOUR DEMO STUFF HERE
targets = np.zeros([2,2])
targets[0,0] = 25
targets[0,1] = 75
targets[1,0] = 50
targets[1,1] = 75
nodes = np.zeros([7,2])
nodes[0,0] = 25
nodes[0,1] = 25
nodes[1,0] = 25
nodes[1,1] = 50
nodes[2,0] = 50
nodes[2,1] = 25
nodes[3,0] = 50
nodes[3,1] = 50
nodes[4,0] = 75
nodes[4,1] = 25
nodes[5,0] = 75
nodes[5,1] = 50
nodes[6,0] = 75
nodes[6,1] = 75
obstacles = np.zeros([5,3])
obstacles[0,0] = 30
obstacles[0,1] = 30
obstacles[0,2] = 4
obstacles[1,0] = 37
obstacles[1,1] = 62
obstacles[1,2] = 6
obstacles[2,0] = 37
obstacles[2,1] = 75
obstacles[2,2] = 5
obstacles[3,0] = 50
obstacles[3,1] = 62
obstacles[3,2] = 3
obstacles[4,0] = 62
obstacles[4,1] = 50
obstacles[4,2] = 4


#build edges
edges = createEdges(nodes, obstacles, targets)
end = time.time()
c = -start + end
print "built my map in ",round(c,3),"seconds"

#run a star
a = aStar(nodes, targets, edges)
print "A* completed in",round(-end+time.time(),3),"seconds"

#plot your nodes, obstacles and A* output
current = targets[1]
for i in a:
		plt.plot([current[0],i[0]],[current[1],i[1]], linewidth = 1, color = 'b')
		current = i
for i in range(len(nodes)):
	circ = plt.Circle((nodes[i,0],nodes[i,1]), radius = .5, color='black')
	ax.add_patch(circ)
for i in range(len(obstacles)):
	circ = plt.Circle((obstacles[i,0],obstacles[i,1]), radius = obstacles[i,2],alpha=0.5, color='blue')
	ax.add_patch(circ)

plt.plot(targets[0,0], targets[0,1],'r^', markersize = 10, label='start')
plt.plot(targets[1,0], targets[1,1],'rs', markersize = 10, label='goal')

# plt.axis(size, aspect=1,'equal')	
plt.axis('equal')
plt.axis(size)

end = time.time()
c = -start + end

reversed_arr = a[::-1]
if len(a) >=1:
	print ''
	print 'My path is:'
	for i in range(len(reversed_arr)):
		print reversed_arr[i]
	print targets[1]

plt.legend()
plt.title('A* Search Implementation')
plt.xlabel('x configuration (m)')
plt.ylabel('y configuration (m)')

plt.show()






