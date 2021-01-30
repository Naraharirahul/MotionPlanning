# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
# Editors: Adithya Suresh and Narahari Rahul Malayanur

import numpy as np
from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
from scene import *
import random
from math import sqrt
import math
disk_robot = True #(change this to False for the advanced extension)
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
    # the roadmap

    graph = Roadmap()

    """
        The below for loop is used to create samples (vertices) where the coordinate x will be spaced equally 
        of length 3 and y will be spaced equally of length 2.5 throughout the entire plot. Both the x and y coordinate
        will be having a random noise ranging from -0.5 to 0.5.
        All these samples are checked first whether they are on the collision free space or not, if they are then
        they get added as vertices. 
    """
    for i in range(0,40):
        x = -48 + 3*i + (random.uniform(-0.5,0.5))
        for j1 in range(0,40):
            y = -48 + 2.5*j1 + (random.uniform(-0.5,0.5))
            coll = collision(x, y)
            if coll is True:
                graph.addVertex((x, y))
    Vertices = graph.getVertices()   # A Vertices list is created with all the vertices being currently placed.
    max_dist = 5  # A maximum distance value is set to connect the vertices between them with a edge.

    """
        The for loop below will add edge between the vertices which are spaced at a distance less than the max_dist 
        variable. Once the edge is added, the interpolate function will check if the edge is passing through the C 
        obstacle space or not. If it is, then the edge gets removed. 
        Once the edge gets removed, for each parent we shall create a list of children and to remove the edges among 
        them to reduce the number of unnecessary edges.  
    """
    for i in Vertices:
        list_vertex = []
        for j in Vertices:
            dist = distance(i,j)
            if dist < max_dist and dist != 0:
                graph.addEdge(i, j, dist)
                some = interpolate(i, j, 0.5)
                if some is False:
                    graph.removeEdge(i,j)
                list_vertex.append(j)
        if len(list_vertex) >= 2:
            for rem in range(0,len(list_vertex)):
                for rem1 in range(0,len(list_vertex)):
                    if rem != rem1:
                        ss = RoadmapVertex.getEdge(list_vertex[rem], list_vertex[rem1].id)
                        if ss is not None:
                            graph.removeEdge(list_vertex[rem],list_vertex[rem1])
    # uncomment this to export the roadmap to a file
    graph.saveRoadmap("prm_roadmap.txt")

    return graph

# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None
def find_path(q_start, q_goal, graph):
    """
        In the following code, we are checking the goal and start coordinates which are generated from random or
        default buttons. We then add these vertices and check their nearest neighbors to add edges between them.
        Once the edges are added, they are checked using the interpolate function to see if any edge is passing through
        the obstacles or not.

        Also, if the start or goal is falling on the obstacle space when a random query is generated, we shall not
        generate path.
    """
    path = []
    goal0 = q_goal[0]
    goal1 = q_goal[1]
    start0 = q_start[0]
    start1 = q_start[1]
    goal = (goal0,goal1)
    start = (start0,start1)

    start_vett = graph.addVertex(start)
    goal_vert = graph.addVertex(goal)
    start_id = RoadmapVertex.getId(start_vett)
    goal_id = RoadmapVertex.getId(goal_vert)
    grids1 = graph.getVertices() # nodes/ vertices

    for grid in grids1:
        if distance(grid, start_vett) < 5:
            graph.addEdge(grid, start_vett, distance(grid, start_vett))
            some = interpolate(grid, start_vett, 0.5)
            if some is False:
                graph.removeEdge(grid, start_vett)
        if distance(grid,goal_vert) < 5:
            graph.addEdge(grid,goal_vert, distance(grid,goal_vert))
            some = interpolate(grid, goal_vert, 0.5)
            if some is False:
                graph.removeEdge(grid, goal_vert)
    grids = graph.getVertices()

    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()

    # Use the PriorityQueue for the open list

    open_set = PriorityQueue(order=min, f=lambda v: v.f)
    """The parent list and the cost for the parent list are created for the A* star path likewise in the third assignment"""
    parent_list = [' ']*len(grids)
    node_cost =  [1000]*len(grids)
     # Path
    f = 0
    g = 0
    h = 0

    """The h value is calculated using the euclidean distance calculation method"""
    for q in range(0, 1):
        h = sqrt((grids[start_id].q[0] - grids[goal_id].q[0]) ** 2 + (grids[start_id].q[1] - grids[goal_id].q[1]) ** 2)
    f = g + h
    """
    Checking whether the collision happens while appending the path and if found any are removed
       Making sure that start and goal configurations are collision-free. Otherwise returning None 
       as mentioned in the comment provided by Dr.Karamouzas
   """
    goal_coll = collision(goal0, goal1)
    start_coll = collision(start0, start1)
    if goal_coll and start_coll is True:
        graph.addVertex((goal0, goal1))
        graph.addVertex((start0, start1))
    else:
        print('Cannot produce path')
        return None

    open_set.put(grids[start_id], Value(f=f, g=g))
    while len(open_set) > 0:
        parent, val = open_set.pop() #parent is the current node here
        children = []
        xlo = RoadmapVertex.getEdges(parent)
        path = []

        """
        The children list is appended finding the IDs between the two nodes at any instant.
           They represent the destination points between the two points and the values are finally updated.
       """
        for child1 in xlo:
            cg = child1.getDestination()
            cg_object = grids[cg]
            if cg_object not in closed_set:
                children.insert(cg_object)

        if parent == grids[goal_id]:
           closed_set.add(parent)
           current_node = parent
           while current_node != grids[start_id]:
               parent_node = parent_list[current_node.id] #Updating the current node with respect to the ID in the parent list that is created
               path.insert(parent_node.q) #Finally updating the correct nodes to the path
               current_node = parent_node
           return path

        closed_set.add(parent)

        for child in children:
            """
            The cost calculation is done by using the euclidean distance calculated g, from the start to the current node
               and h, from the current node to the end node. The resulting f value which is the sum of both g and h are taken.
           """
            g_val = distance(child, parent) + val.g
            h_val = distance(parent, grids[goal_id])
            f_val = g_val  + h_val
            cost_child = f_val
            child_new = g_val
            if child not in open_set or closed_set:
                open_set.put(child, Value(f=cost_child, g=child_new))

            if child in open_set:
                cost_need = open_set.get(child)
                cost_f = cost_need.f
                if cost_child < cost_f:
                    open_set.put(child, Value(f=cost_child, g=child_new))

            """
            Backtracking the parent list with respect to the cost value from the goal node to form the optimal path
            """
            if node_cost[child.id] > cost_child:
               node_cost[child.id] = cost_child
               parent_list[child.id] = parent

    return path
# ----------------------------------------
# below are some functions that you may want to populate/modify and use above
# ----------------------------------------

def nearest_neighbors(graph, q, max_dist=10.0):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances
    """
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
    """
    for q in range(0,1):
        distance_new = sqrt((q1.q[0] -q2.q[0])**2 + (q1.q[1] - q2.q[1])**2)
        return distance_new

def collision(x,y):
    """
        Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.

        Using the obstacles list, it checks whether the current vertex lies on the obstacle space or not.
    """

    for j in range(0, 26):
        xs = []
        ys = []
        for point in obstacles[j].points:
            count = 0
            xs.append(point[0])
            ys.append(point[1])
            x_min = min(xs) - robot_radius - 0.1
            x_max = max(xs) + robot_radius + 0.1
            y_min = min(ys) - robot_radius - 0.1
            y_max = max(ys) + robot_radius + 0.1
        if x_min <= x <= x_max and y_min <= y <= y_max:
            count = 1
            break
    if count != 1:
        return True

def interpolate (q1, q2, stepsize):
    """
        Returns an interpolated local path between two given configurations.
        It can be used to determine whether an edge between vertices is collision-free.

        Using the below code, we can check whether the edge intersects the C obstacle space or not.
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
