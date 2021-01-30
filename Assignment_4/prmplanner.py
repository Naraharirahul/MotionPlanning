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

    obstacles = scene_obstacles # setting the global obstacle variable
    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot
    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)
    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box

    robot_radius = max(robot_width, robot_height)/2.
    A = np.zeros((1000,2))
    # the roadmap
    graph = Roadmap()
    list_app = []
    list_app = []
    vertices_app = []

    goof = graph.addVertex((14.378, -4.328))
    goof1 = graph.addVertex((-44.184, -26.821))
    #graph.addVertex((14.378,-4.328))
    for i in range(0,33):
        # x = int(np.random.uniform(-50,50))
        # y = int(np.random.uniform(-50,50))
        x = -45+ 3*i + (random.uniform(-0.5,0.5))
        for j1 in range(0,33):
            y = -45 + 3*j1 + (random.uniform(-0.5,0.5))
            A[i][0] = x
            A[i][1] = y
            for j in range(0, 26):
                xs = []
                ys = []
                for point in obstacles[j].points:
                    count = 0
                    xs.append(point[0])
                    ys.append(point[1])
                    x_min = min(xs) - 2
                    x_max = max(xs) + 2
                    y_min = min(ys) - 2
                    y_max= max(ys) + 2
                if x_min <= x <= x_max and y_min <= y <= y_max:
                    count = 1
                    break
            if count != 1:
                graph.addVertex((x, y))
    # for qq in range(len(A)):



    Vertices = graph.getVertices()

    max_dist = 5
    for i in Vertices:
        list_vertex = []
        for j in Vertices:
            dist = distance(i,j)
            # print('in loop dist', dist)
            if dist < max_dist and dist != 0:
                edge_check = graph.addEdge(i, j, dist)
                some = interpolate(i, j, 0.5)
                # print('some',some)
                if some is False:
                    graph.removeEdge(i,j)
                list_a_x = i.id
                list_a_y = j.id
                list_a = (list_a_x, list_a_y)
                list_app.append(list_a)
                list_b = (list_a_y, list_a_x)
                if list_b not in list_app:
                    graph.removeEdge(i, j)
                list_vertex.append(j)
        if len(list_vertex) >= 2:
            for rem in range(0,len(list_vertex)):
                for rem1 in range(0,len(list_vertex)):
                    if rem != rem1:
                        ss = RoadmapVertex.getEdge(list_vertex[rem], list_vertex[rem1].id)
                        if ss is not None:
                            graph.removeEdge(list_vertex[rem],list_vertex[rem1])
    # p_neighbour = []
    # c_neighbour = []
    #
    # for x in range(len(Vertices)):
    #     p_neighbour.clear()
    #     for z in range(len(Vertices)):
    #         if not x == z:
    #             parent = RoadmapVertex.getEdge(Vertices[x], Vertices[z].id)
    #             if not parent is None:
    #                 p_neighbour.append(Vertices[z])
    #         c_neighbour.clear()
    #         for u in range(len(p_neighbour)):
    #             for h in range(len(Vertices)):
    #                 if not u == h:
    #                     children = RoadmapVertex.getEdge(p_neighbour[u], Vertices[h].id)
    #                     if not children is None:
    #                         c_neighbour.append(Vertices[h])
    #
    #             for r in range(len(p_neighbour)):
    #                 for m in range(len(c_neighbour)):
    #                     if p_neighbour[r] == c_neighbour[m]:
    #                         graph.removeEdge(Vertices[x], Vertices[z])
    #                         break
    aa = RoadmapVertex.getEdges(goof1)
    #print('build',RoadmapVertex.getConfiguration(goof1))
    # uncomment this to export the roadmap to a file
    # graph.saveRoadmap("prm_roadmap.txt")

    return graph

# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None
def find_path(q_start, q_goal, graph):
    path = []
    trial = []

    # print('goal',q_goal)
    # print('start',q_start)
    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use the PriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)
    grids = graph.getVertices() # nodes/ vertices
    print(len(grids))

     # Path
    f = 0
    g = 0
    h = 0
    for q in range(0, 1):
        h = sqrt((grids[0].q[0] - grids[1].q[0]) ** 2 + (grids[0].q[1] - grids[1].q[1]) ** 2)

    f = g + h
    open_set.put(grids[0], Value(f=f, g=g))
    while len(open_set) > 0:
        parent, val = open_set.pop()
        closed_set.add(parent)
        new = (RoadmapVertex.getConfiguration(parent))
        new1= new[0]
        new2 = new[1]
        trial=[new1,new2]

        if parent == grids[1]:
           break
        else:
            for child in grids:
                sssss = RoadmapVertex.getEdge(parent, child.id)
                if sssss is not None:
                    if child not in closed_set:
                        g_val = distance(child, parent) + val.g
                        if child not in open_set or open_set.get(child).g > g_val:
                            h_val = distance(parent, grids[1])
                            # print('The H value is:',h_val)
                            f_val = g_val  + h_val
                            open_set.put(child, Value(f=f_val, g=g_val))
        # print('The trial is:', trial)
        path.append(trial)
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

def distance (q1, q2): 
    """
        Returns the distance between two configurations. 
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration  
    """

    for q in range(0,1):
        distance_new = sqrt((q1.q[0] -q2.q[0])**2 + (q1.q[1] - q2.q[1])**2)
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
    for j1 in range(0,26):
        xs1 = []
        ys1 = []
        for p in obstacles[j1].points:
            xs1.append(p[0])
            ys1.append(p[1])
        for r in range(0,3):
            ta = ((ys1[r] - ys1[r+1])*(q1.q[0]- xs1[r]) + (xs1[r+1]- xs1[r])*(q1.q[1]-ys1[r]))/(0.00001 + (xs1[r+1] - xs1[r])*(q1.q[1] - q2.q[1]) - (q1.q[0] - q2.q[0])*(ys1[r+ 1]- ys1[r]))
            tb = ((q1.q[1] - q2.q[1])*(q1.q[0]- xs1[r]) + (q2.q[0]- q1.q[0])*(q1.q[1] - ys1[r]))/( 0.0001 + (xs1[r+1] - xs1[r])*(q1.q[1] - q2.q[1]) - (q1.q[0]- q2.q[0])*(ys1[r+1]- ys1[r]))
            if 0 <= ta <= 1 and 0 <= tb <= 1:
               return False

if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
