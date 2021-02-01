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
from math import pi
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

def build_roadmap(q_range, robot_dim, scene_obstacles):
    global obstacles, robot_width, robot_height, robot_radius, VerList

    obstacles = scene_obstacles  # setting the global obstacle variable

    x_limit = q_range[0]  # the range of x-positions for the robot
    y_limit = q_range[1]  # the range of y-positions for the robot
    theta_limit = q_range[2]  # the range of orientations for the robot (advanced extension)

    robot_width, robot_height = robot_dim[0], robot_dim[1]  # the dimensions of the robot, represented as an oriented bounding box
    robot_radius = max(robot_width, robot_height) / 2.

    # the roadmap
    graph = Roadmap()

    # uncomment this to export the roadmap to a file
    graph.saveRoadmap("prm_roadmap.txt")

    count = 0
    points = []
    VerList = []

    for i in range(0, 20):
        xo = 1 + i*5
        for j in range(0, 20):
            yo = 1 + j*5
            theta = [pi/4, 3*pi/4, 5*pi/4, 7*pi/4]
            graph.addVertex((xo, yo))
            for k in range(len(theta)):
                r = 5
                x1 = xo + r * math.cos(theta[k])
                y1 = yo + r * math.sin(theta[k])
                graph.addVertex((x1, y1))
    VerList = graph.getVertices()

    return graph

    # The sampled configurations are not random but the vertices form a square. In this sampled space, according to
    # the distance given, an edge never passes over the obstacle but still we tried to implement subdivision method
    #below.
#     for k in range(len(VerList)):
#         for p in range(len(VerList)):
#             if not k == p:
#                 dist = distance(VerList[k], VerList[p])
#                 if dist < 5:
#                     graph.addEdge(VerList[k], VerList[p], dist)
#
#                     x1 = RoadmapVertex.getConfiguration(VerList[k])[0]
#                     x2 = RoadmapVertex.getConfiguration(VerList[p])[0]
#                     y1 = RoadmapVertex.getConfiguration(VerList[k])[1]
#                     y2 = RoadmapVertex.getConfiguration(VerList[p])[1]
#
#                     midx = (x1 + x2) / 2
#                     midy = (y1 + y2) / 2
#
#                     midx1 = (x1 + x2) / 2
#                     midy1 = (y1 + y2) / 2
#
#                     check_collision = collision(midx,midy)
#                     if check_collision is not False:
#                         graph.removeEdge(VerList[k], VerList[p])
#
#                     for i in range(0, 5):
#                         sdx = (midx + x1) / 2
#                         sdy = (midy + y1) / 2
#
#                         sdx1 = (midx1 + x2) / 2
#                         sdy1 = (midy1 + y2) / 2
#
#                         midx = sdx
#                         midy = sdy
#                         midx1 = sdx1
#                         midy1 = sdy1
#
#                         check_collision = collision(sdx, sdy)
#
#                         if check_collision is not False:
#                             graph.removeEdge(VerList[k], VerList[p])
#
#                         check_collision = collision(sdx1,sdy1)
#                         if check_collision is not False:
#                             graph.removeEdge(VerList[k], VerList[p])
#
#     p_neighbour = []
#
#     #Removing the edges between the children of a parent which ensures that NO TRIANGLES are formed.
#     for xi in range(len(VerList)):
#         p_neighbour.clear()
#         for zi in range(len(VerList)):
#             if not xi == zi:
#                 parent = RoadmapVertex.getEdge(VerList[xi], VerList[zi].id)
#                 if not parent is None:
#                     p_neighbour.append(VerList[zi])
#                 for ki in range(len(p_neighbour)):
#                     for yi in range(len(p_neighbour)):
#                         if not ki == yi:
#                             chil = RoadmapVertex.getEdge(p_neighbour[ki], p_neighbour[yi].id)
#                             if not chil is None:
#                                 graph.removeEdge(p_neighbour[ki], p_neighbour[yi])
#     return graph
# # ----------------------------------------
# # modify the code below
# # ----------------------------------------
# # Query phase: Connect start and goal to roadmap and find a path using A*
# # (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# # The returned path should be a list of configurations, including the local paths along roadmap edges
# # Make sure that start and goal configurations are collision-free. Otherwise return None

def find_path(q_start, q_goal, graph):
    path = []
    goalx = q_goal[0]
    startx = q_start[0]
    goaly = q_goal[1]
    starty = q_start[1]

    goal = (goalx, goaly)
    start = (startx, starty)

    UpdatedVerList = graph.getVertices()

    # A* algorithm:
    closed_set = OrderedSet()
    open_set = PriorityQueue(order=min, f=lambda v: v.f)

    h = distance(startVer, goalVer)
    g = 0
    f = g + h
    next = startVer
    open_set.put(next, Value(f=f, g=g))

    parent = {}
    check = []

    while len(open_set) > 0:
        next, value = open_set.pop()
        closed_set.add(next)
        nextId = next.id
        gn = value.g

        #Print the path after it reaches the goal and traceback to the start point
        if nextId == goalId:
            while not nextId == startId:
                nextId = parent[nextId]
                ee = RoadmapVertex.getConfiguration(UpdatedVerList[nextId])
                path.insert(0, ee)
            break
        else:
            neighbour = RoadmapVertex.getEdges(next)
            check.append(nextId)
            for ji in range(len(neighbour)):
                piId = neighbour[ji].id
                if piId not in check:
                    parent.update({piId: nextId})
                    g = distance(next, UpdatedVerList[piId]) + gn
                    h = distance(UpdatedVerList[piId], goalVer)
                    f = g + h
                    if UpdatedVerList[piId] not in closed_set:
                        open_set.put(UpdatedVerList[piId], Value(f=f, g=g))
                    else:
                        pass
                else:
                    pass
    return path

# # ----------------------------------------
# # below are some functions that you may want to populate/modify and use above
# # ----------------------------------------
# def nearest_neighbors(graph, q1, q2, max_dist=10.0):
#     """
#         Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
#         You may also want to return the corresponding distances
#     """
#
#     return None
#
# def k_nearest_neighbors(graph, q, K=4):
#     """
#         Returns the K-nearest roadmap vertices for a given configuration q.
#         You may also want to return the corresponding distances
#     """
#     return None
#
# def distance(q1, q2):
#     """
#         Returns the distance between two configurations.
#         You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration
#     """
#     eucDis = sqrt((q1.q[0] - q2.q[0]) ** 2 + (q1.q[1] - q2.q[1]) ** 2)
#     return eucDis
#
# def collision(xc, yc):
#     """
#         Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.
#     """
#     count = 0
#     for e in range(0, 26):
#         xmin = obstacles[e].x_min - robot_radius
#         ymin = obstacles[e].y_min - robot_radius
#         xmax = obstacles[e].x_max + robot_radius
#         ymax = obstacles[e].y_max + robot_radius
#
#         if not (xmin < xc < xmax and ymin < yc < ymax):
#             count = count + 1
#     if count == len(obstacles):  # ensures that configuration is outside of all the obstacles
#         return False
#
# def interpolate(q1, q2, stepsize):
#     """
#         Returns an interpolated local path between two given configurations.
#         It can be used to determine whether an edge between vertices is collision-free.
#     """
#     return None

if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()

