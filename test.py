from scipy.spatial import KDTree
import numpy as np

x_points = [10,15,20,25,30]
y_points = [10,15,20,25,30]

tree = KDTree(np.vstack((x_points,y_points)).T)

print(tree.data)

dist, index = tree.query([19,19])

print(dist)

print(index)

print(x_points[index]," ",y_points[index])


