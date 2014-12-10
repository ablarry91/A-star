from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from shapely.geometry import MultiPoint
import numpy as np

poly = np.matrix([[79.99, 78.66],[-9.32,93.81],[66.29,-2.07],[-23.02,13.08]])

# poly = Polygon([(79.99, 78.66),(-9.32,93.81),(66.29,-2.07),(-23.02,13.08)])
# poly = MultiPoint(poly).convex_hull
# poly = Polygon(poly)
point = Point(76.56,63.58)

# print poly.bounds
# print poly.contains(point)
# print point


test = np.array([5,5])
# print hash(str(test))

testDict = {}
# testDict[hash(str(test))] = np.array([5,5])
# print testDict
# print testDict[hash(str(test))]

# for i in poly:
	# testDict[hash(str(i))]

keys = [1,2,3,5,6,7]
# print {key: None for hash(str(key)) in poly}
# print testDict.fromkeys([hash(str(poly))])

# for i in poly:
# 	testDict[hash(str(i))] = np.zeros([1,2])


# testDict[hash(str(poly[0]))] = 5
# testDict[hash(str(poly[0]))] = 5
# testDict[hash(str(poly[0]))].append(poly[1])
# testDict[hash(str(poly[0]))].append(poly[2])

for i in range(2):
	try:
		testDict[hash(str(poly[0]))] = np.vstack([testDict[hash(str(poly[0]))],poly[1]])
	except:
		testDict[hash(str(poly[0]))] = poly[1]
print testDict
# print testDict

list1=[1,2,3,4,5]
list2=[123,234,456]
d={'a':[],'b':[]}
d['a'].append(list1)
d['a'].append(list2)
# print d['a']




# [ 76.56251819  63.56733501  45.29309168] 
# [[ 79.98603216  78.65994162]
#  [ -9.32420212  93.81047527]
#  [ 66.29024379  -2.07477677]
#  [-23.01999049  13.07575688]] 
# (array([ 35.33091502,  86.23520845]), array([ 21.63512665,   5.50049006])) 

