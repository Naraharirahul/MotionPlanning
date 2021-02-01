# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

import numpy as np
from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
import random
from math import sqrt
from math import *
import math
disk_robot = False #(change this to False for the advanced extension)


obstacles = None # the obstacles 
robot_radius = None # the radius of the robot
robot_width = None # the width of the OBB robot (advanced extension)
robot_height = None # the height of the OBB robot (advanced extension)


# ----------------------------------------
# modify the code below
# ----------------------------------------

# Construction phase: Build the roadmap
# You should incrementally sample configurations according to a strategy and add them to the roadmap, 
# select the neighbors of each sample according to a distance function and strategy, and
# attempt to connect the sample to its neighbors using a local planner, leading to corresponding edges
# See graph.py to get familiar with the Roadmap class
distance_app = []
def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius
    graph = Roadmap()
    directions = [0, 1, 2, 3, 4, 5, 6, 7]
    angle = [0, pi/4, pi/2, 3*pi/4, pi, 5*pi/4, 2*pi]
    direction_name= ['N', 'NW', 'W', 'SW', 'S', 'SE', 'E', 'NE']
    action = [-1, 0, 1]
    # l = 1  # Length
    # forward = [[0, l],                   #0: go N
    #            [-l/sqrt(2), l/sqrt(2)]   #1: go NW
    #            [0, -l],                  #2: go W
    #            [-l/sqrt(2), -l/sqrt(2)]  #3 go SW
    #            [0, -l],                  #4: go S
    #            [l/sqrt(2), -l/sqrt(2)]   #5: go SE
    #            [l, 0]                    #6: go E
    #            [l/sqrt(2), l/sqrt(2)]]   #7 go NE
    action_name = ['R', 'F', 'L']


    cost = [1, 1, 1]  # corresponding cost values
    graph = Roadmap()

    obstacles = scene_obstacles # setting the global obstacle variable
    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot
    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)
    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box
    robot_radius = max(robot_width, robot_height) / 2.

    for i in range(0, 10):
        xo = 1 + i * 5
        for j in range(0, 10):
            yo = 1 + j * 5
            xon = xo*cos(math.radians(0.1)) - yo*sin(math.radians(0.1))
            yon = yo*cos(math.radians(0.1)) + xo*sin(math.radians(0.1))
            theta = [45.1, 135.1, 225.1, 315.1]
            graph.addVertex((xon, yon))
            # theta = [90]
            for k in range(len(theta)):
                r = 5
                x1 = xo + r * math.cos(math.radians(theta[k]))
                y1 = yo + r * math.sin(math.radians(theta[k]))
                x1n = x1*cos(math.radians(0.1)) - y1*sin(math.radians(0.1))
                y1n = y1*cos(math.radians(0.1)) + x1*sin(math.radians(0.1))
                # print('x1', x1n,y1n,theta)
                # print('y1',y1n)
                graph.addVertex((x1n, y1n))



    # Vertices = graph.getVertices()
    # max_dist = 5
    # for i in Vertices:
    #     for j in Vertices:
    #         print(i[0],i[1],j[0],j[1])
    #         # dist = distance(i[0],j[0],i[1],j[1])
    #         # if dist < max_dist:
    #         #     graph.addEdge(i, j, dist)


    return graph
    #print('build',RoadmapVertex.getConfiguration(goof1))
    # uncomment this to export the roadmap to a file
    # graph.saveRoadmap("prm_roadmap.txt")

# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None
def find_path(q_start, q_goal, graph):
    path = []
    parent = {}
    check = []
    l = 1  # Length of car
    q_start = (0.9912733581705093,15.999992384566438,90.1)
    # q_goal = (0.9912733581705093,15.999992384566438,90.1)
    # q_goal = (0.973820074511528 ,15.99997715369931,90.1)
    q_goal = (6.23884259438487, 13.18071969726346, -44.900000000000006)
    # q_goal = (4.110332424479485, 17.298331363126078, -44.900000000000006)
    x = q_start[0]
    y= q_start[1]
    theta = q_start[2]
    closed_set = OrderedSet()
    open_set = PriorityQueue(order=min,f=lambda v: v.f)
    g = 0
    h = 3 #distance(q_start,q_goal)
    f = g + h
    open_set.put(q_start, Value(f=f, g=g))
    count = 0
    while len(open_set) > 0:
        neighbour = []
        next, value = open_set.pop()
        closed_set.add(next)
        # path.append(next)
        # print(next)
        g = value.g
        x = next[0]
        y = next[1]
        theta = next[2]
        if next == q_goal:
        #     while not (x == q_start[0] and y == q_start[1] and theta == q_start[2]):
            while not next == q_start:
                next = parent[next]
                # print(next[0])
                # path.insert(0,next)
                print(next)
                path.insert(0,(next[0],next[1],math.radians(next[2])))
            break
        else:
            beta = 45
            ycl = y + l * math.cos(math.radians(theta))     # Left center
            xcl = x + math.tan(math.radians(theta)) * (y - ycl)

            # left child coordinates:
            ynl = ycl + l * math.sin(math.radians(theta - beta))
            xnl = xcl - (ycl - ynl) / math.tan(math.radians(theta - beta))
            # print("left", xnl, ynl)
            gamaL = theta + beta
            neighbour_l = (xnl,ynl,gamaL)
            neighbour.append(neighbour_l)

            ycr = y - l * math.cos(math.radians(theta))     # Right center
            xcr = x + math.tan(math.radians(theta)) * (y - ycr)

            # rightChild coordinates:
            ynr = ycr + l * math.sin(math.radians(theta + beta))
            xnr = xcr - (ycr - ynr) / math.tan(math.radians(theta + beta))
            # print("right", xnr, ynr)
            gamaR = theta - beta
            neighbour_r = (xnr,ynr,gamaR)
            neighbour.append(neighbour_r)

            # straight child coordinates
            yns = y + l * math.sin(math.radians(theta))
            xns = x - (y - yns) / math.tan(math.radians(theta))
            gamaS = theta
            neighbour_s = (xns,yns,gamaS)
            neighbour.append(neighbour_s)
            count = count + 1
            print('c',count)
            for i in neighbour:
                x_child = i[0]
                y_child = i[1]
                theta_child = i[2]
                check.append(next)
                # if not (x == q_start[0] and y == q_start[1] and theta == q_start[2]):
                if i not in check:
                    # setdefault(i, q_start)
                    parent.update({i: next})
                    # print(parent)
                # print(parent)
                print(x_child,y_child,theta_child)
                if i not in closed_set:
                    g_child = g + distance(x,y,x_child,y_child)
                # print('g', g_child)
                if i not in open_set or open_set.get(i).g > g_child:
                    h_child = distance(x_child,y_child,q_goal[0],q_goal[1])
                    # print(h_child)
                    f_child = g_child + h_child
                    open_set.put(i,Value(f_child,g_child))

        # if p
    return path
# ----------------------------------------
# below are some functions that you may want to populate/modify and use above 
# ----------------------------------------

def nearest_neighbors(graph, q, max_dist=10.0):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances 
    """

    RoadmapVertex.getConfiguration(q)
    return None

def k_nearest_neighbors(graph, q, K=10):
    """
        Returns the K-nearest roadmap vertices for a given configuration q. 
        You may also want to return the corresponding distances 
    """

    return None

def distance (m1, n1, m2, n2):
    """
        Returns the distance between two configurations. 
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration  
    """


    distance_new = sqrt((m1 - m2)**2 + (n1 - n2)**2)
    return distance_new

def collision(q):
    """
        Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.  
    """

def interpolate (q1, q2, stepsize):
    """
        Returns an interpolated local path between two given configurations. 
        It can be used to determine whether an edge between vertices is collision-free. 
    """
    # for j1 in range(0,26):
    #     xs1 = []
    #     ys1 = []
    #     for p in obstacles[j1].points:
    #         xs1.append(p[0])
    #         ys1.append(p[1])
    #     for r in range(0,3):
    #         ta = ((ys1[r] - ys1[r+1])*(q1.q[0]- xs1[r]) + (xs1[r+1]- xs1[r])*(q1.q[1]-ys1[r]))/(0.00001 + (xs1[r+1] - xs1[r])*(q1.q[1] - q2.q[1]) - (q1.q[0] - q2.q[0])*(ys1[r+ 1]- ys1[r]))
    #         tb = ((q1.q[1] - q2.q[1])*(q1.q[0]- xs1[r]) + (q2.q[0]- q1.q[0])*(q1.q[1] - ys1[r]))/( 0.0001 + (xs1[r+1] - xs1[r])*(q1.q[1] - q2.q[1]) - (q1.q[0]- q2.q[0])*(ys1[r+1]- ys1[r]))
    #         if 0 <= ta <= 1 and 0 <= tb <= 1:
    #            return False

if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
