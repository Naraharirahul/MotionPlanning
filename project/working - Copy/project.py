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

distance_app = []
def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius
    graph = Roadmap()
    obstacles = scene_obstacles
    robot_width, robot_height = robot_dim[0], robot_dim[1]

    # Here we have just specified the start coordinate of the vehicle.

    graph.addVertex((3,5))

    return graph

def find_path(q_start, q_goal, graph):
    path = []
    parent = {}
    check = []

    # The turning radius of the vehicle is calculated based on the realistic values.
    l = 4.5  # Turning radius in m.
    beta = 30 # Minimum steering angle in degrees
    x_new = 0
    y_new = 0
    closed_set = OrderedSet()
    open_set = PriorityQueue(order=min, f=lambda v: v.f)
    g = 0
    h = distance(q_start[0],q_goal[0],q_start[1],q_goal[1])
    f = g + h
    x_s = q_start[0]
    y_s = q_start[1]
    theta_s = 90 - math.degrees(q_start[2])
    q_start_d = (x_s, y_s, theta_s)
    x_g = q_goal[0]
    y_g = q_goal[1]
    theta_g = 90 - math.degrees(q_goal[2])
    q_goal_d =(x_g,y_g,theta_g)

    # Above we have converted the orientation which is in radians to degrees for our calculation purpose

    open_set.put(q_start_d, Value(f=f, g=g))

    while len(open_set) > 0:
        neighbour = []
        next, value = open_set.pop()
        closed_set.add(next)
        g = value.g
        x = next[0]
        y = next[1]
        theta = next[2]
        count = 0
        if next == q_goal_d:  # This statement will be executed if the chosen goal is reached with the kinematic constraints.
            while not next == q_start_d:
                path.insert(1, (next[0], next[1], math.radians(90 - next[2])))
                graph.addVertex((next[0], next[1]))
                next = parent[next]
                count = 0
            break

        # The below elif statement is run when the vehicle is doing parallel parking. Below we shall try to get as
        # close as possible to the goal configuration with the same orientation as that of the goal position. Once it
        # gets close, we shall try to make the car align parallelly to the goal posstion by moving little front or
        # back. After that, the path taken by the car to reach this point is then inserted in path.

        elif (next[2]==q_goal_d[2]) and (distance(x,y,q_goal_d[0],q_goal_d[1])<3):
            if round(x) != round(q_goal_d[0]) or round(y) != round(q_goal_d[1]):
                if round(theta) == 90 or round(theta) == -90:
                    x_new = x
                    y_new = q_goal_d[1]
                else:
                    x_new = q_goal_d[0]
                    y_new = y
                path.insert(0, (x_new, y_new,  math.radians(90 - next[2])))

                # Distance is calculated between the goal and the current orientation
                d = distance(x_new, y_new, q_goal_d[0], q_goal_d[1])
                count = 1
                while not next == q_start_d:
                    path.insert(1, (next[0], next[1], math.radians(90 - next[2])))
                    next = parent[next]
                break

        # Below part of the code shall try to get the nodes of the children it can reach using its fixed steering
        # angle and turning radius. The cost of each node expanded is calculated and then inserted in the open set.

        else:
            # right center is calculated based on the steering angle and turning radius
            ycl = y + l * math.cos(math.radians(theta))
            xcl = x + math.tan(math.radians(theta)) * (y - ycl)

            # right child coordinates are then calculated from the right center
            ynl = ycl + l * math.sin(math.radians(theta - beta))
            xnl = xcl - (ycl - ynl) / math.tan(math.radians(theta - beta))
            gamaL = theta + beta
            neighbour_l = (xnl,ynl,gamaL)
            # Checks whether the point lies on the collision free space or not
            if collision(xnl, ynl) is True:
                neighbour.append(neighbour_l)

            # left center is calculated based on the steering angle and turning radius
            ycr = y - l * math.cos(math.radians(theta))
            xcr = x + math.tan(math.radians(theta)) * (y - ycr)

            # left child coordinates are then calculated from the left center
            ynr = ycr + l * math.sin(math.radians(theta + beta))
            xnr = xcr - (ycr - ynr) / math.tan(math.radians(theta + beta))
            gamaR = theta - beta
            neighbour_r = (xnr,ynr,gamaR)

            # Checks whether the point lies on the collision free space or not
            if collision(xnr,ynr) is True:
                neighbour.append(neighbour_r)

            # straight child coordinates
            yns = y + l * math.sin(math.radians(theta))
            xns = x - (y - yns) / math.tan(math.radians(theta))
            gamaS = theta
            neighbour_s = (xns,yns,gamaS)

            # Checks whether the point lies on the collision free space or not
            if collision(xns, yns) is True:
                neighbour.append(neighbour_s)

            for i in neighbour:
                x_child = i[0]
                y_child = i[1]
                check.append(next)
                if i not in check:
                    parent.update({i: next})
                if i not in closed_set:
                    g_child = g + distance(x,y,x_child,y_child)
                if i not in open_set or open_set.get(i).g > g_child:
                    h_child = distance(x_child,y_child,q_goal_d[0],q_goal_d[1])
                    f_child = g_child + h_child
                    open_set.put(i,Value(f_child,g_child))

    # Below is the code which calculates the angle required to maneuver the vehicle to reach the goal position which
    # is at a specific distance. The below code is only executed when the car is unable to raach a specific coordinate.

    if count == 1:
        alpha1 = acos(l/(l + d/2))
        alpha = math.degrees(alpha1)

        # The below part of the code checks whether the parking space is on the right side of the vehicle or to the
        # left side. Here it has been calculated by checking the x or y difference from the current and goal positions.

        if q_goal_d[0] != x_new:
            if q_goal_d[0] > x_new:
                right_parking(x_new, y_new, alpha, path, q_goal_d, d, l)
            else:
                left_parking(x_new, y_new, alpha, path, q_goal_d, d, l)
        else:
            if q_goal_d[1] > y_new:
                right_parking(x_new, y_new, alpha, path, q_goal_d, d, l)
            else:
                left_parking(x_new, y_new, alpha, path, q_goal_d, d, l)
    return path

def left_parking(x_new,y_new,alpha,path,q_goal_d,d,l):

    # The below code is implementing the type one maneuver if the parking space is on the left side of the vehicle.

    plcy = y_new + l * math.cos(math.radians(q_goal_d[2]))  # right center
    plcx = x_new + math.tan(math.radians(q_goal_d[2])) * (y_new - plcy)

    # right child coordinates:
    plny = plcy + l * math.sin(math.radians(q_goal_d[2] - alpha))
    plnx = plcx - (plcy - plny) / math.tan(math.radians(q_goal_d[2] - alpha))
    dia = 2 * (l + d / 2) * sin(math.radians(90 + alpha))

    prcy = plny - dia * math.cos(math.radians(90 - (q_goal_d[2] - alpha)))  # left center
    prcx = plnx + math.tan(math.radians(90 - (q_goal_d[2] - alpha))) * (plny - prcy)

    path.insert(0, (plnx, plny, math.radians(90 - (q_goal_d[2] + alpha))))  # Point 2
    path.insert(0, (prcx, prcy, math.radians(90 - (q_goal_d[2] + alpha))))  # Point 3

    path.insert(0, (q_goal_d[0], q_goal_d[1], math.radians(90 - q_goal_d[2]))) # Point 4

def right_parking(x_new,y_new,alpha,path,q_goal_d,d,l):

    # The below is for implementing the type 1 maneuver when the parking space is on the right side of the vehicle

    rcy = y_new - l * math.cos(math.radians(q_goal_d[2]))  # Right center
    rcx = x_new + math.tan(math.radians(q_goal_d[2])) * (y_new - rcy)

    rny = rcy + l * math.sin(math.radians(q_goal_d[2] + alpha))
    rnx = rcx - (rcy - rny) / math.tan(math.radians(q_goal_d[2] + alpha))
    dia = 2 * (l + d / 2) * sin(math.radians(90 + alpha))
    lcy = rny + dia * math.cos(math.radians(90 + (q_goal_d[2] - alpha)))  # Left center
    lcx = rnx + math.tan(math.radians(90 + (q_goal_d[2] - alpha))) * (rny - lcy)

    path.insert(0, (rnx, rny, math.radians(90 - (q_goal_d[2] - alpha))))  # Point 2
    path.insert(0, (lcx, lcy, math.radians(90 - (q_goal_d[2] - alpha))))  # Point 3
    path.insert(0, (q_goal_d[0], q_goal_d[1], math.radians(90 - q_goal_d[2]))) # Point 4

def distance (m1, n1, m2, n2):

    # This is to calculate the distance between two points where m1,n1 are x,y of the first coordinate and m2,
    # n2 are the x,y of the second coordinate.

    distance_coord = sqrt((m1 - m2)**2 + (n1 - n2)**2)
    return distance_coord

def collision(x,y):

    # This fucntion is used to check if the given configuration of the vehicle lies on the obstacle space or not.

    for j in range(0, 13):
        xs = []
        ys = []
        for point in obstacles[j].points:
            value = 0
            xs.append(point[0])
            ys.append(point[1])
            x_min = min(xs) - robot_height
            x_max = max(xs) + robot_height
            y_min = min(ys) - robot_height
            y_max = max(ys) + robot_height
        if x_min <= x <= x_max and y_min <= y <= y_max:
            value = 1
            break
    if value != 1:
        return True

if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
