# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

from graph import RoadmapVertex, RoadmapEdge, Roadmap
from obstacles import BoxObstacle
from utils import *
import random
import math
from math import sqrt
from scene import Scene

disk_robot = True  # (change this to False for the advanced extension)

obstacles = None  # the obstacles
robot_radius = None  # the radius of the robot
robot_width = None  # the width of the OBB robot (advanced extension)
robot_height = None  # the height of the OBB robot (advanced extension)
# ----------------------------------------
# modify the code below
# ----------------------------------------

# Construction phase: Build the roadmap
# You should incrementally sample configurations according to a strategy and add them to the roadmap,
# select the neighbors of each sample according to a distance function and strategy, and
# attempt to connect the sample to its neighbors using a local planner, leading to corresponding edges
# See graph.py to get familiar with the Roadmap class

count = 0
points=[]
for i in range(10,30):
    for j in range(10,30):
        xo, yo = i, j
        theta = [45, 135, 225, 315]
        for k in range(len(theta)):
            r = 5
            x1 = xo+r*math.cos(theta[k])
            y1 = yo+r*math.sin(theta[k])

            # if (x1, y1) not in points:
            #     points.append((x1, y1))
            #     #print("kshf")
            #     count = count + 1

# print(count)







