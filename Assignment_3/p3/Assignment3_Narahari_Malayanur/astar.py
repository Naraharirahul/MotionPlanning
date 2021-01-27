# astar.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#


# Compute the optimal path from start to goal.
# The car is moving on a 2D grid and
# its orientation can be chosen from four different directions:
forward = [[-1,  0], # 0: go north
           [ 0, -1], # 1: go west
           [ 1,  0], # 2: go south
           [ 0,  1]] # 3: go east

# The car can perform 3 actions: -1: right turn and then move forward, 0: move forward, 1: left turn and then move forward
action = [-1, 0, 1]
action_name = ['R', 'F', 'L']
cost = [1, 1, 10] # corresponding cost values

# GRID:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = (4, 3, 0) # (grid row, grid col, orientation)
                
goal = (2, 0, 1) # (grid row, grid col, orientation)


heuristic = [[2, 3, 4, 5, 6, 7], # Manhattan distance
        [1, 2, 3, 4, 5, 6],
        [0, 1, 2, 3, 4, 5],
        [1, 2, 3, 4, 5, 6],
        [2, 3, 4, 5, 6, 7]]

from utils import (Value, OrderedSet, PriorityQueue)

"""
Two data structures are provided for your open and closed lists: 

 1. OrderedSet is an ordered collection of unique elements.
 2. PriorityQueue is a key-value container whose `pop()` method always pops out
    the element whose value has the highest priority.

 Common operations of OrderedSet, and PriorityQueue
   len(s): number of elements in the container s
   x in s: test x for membership in s
   x not in s: text x for non-membership in s
   s.clear(): clear s
   s.remove(x): remove the element x from the set s;
                nothing will be done if x is not in s

 Unique operations of OrderedSet:
   s.add(x): add the element x into the set s
   s.pop(): return and remove the LAST added element in s;

 Example:
   s = Set()
   s.add((0,1,2))    # add a triplet into the set
   s.remove((0,1,2)) # remove the element (0,1,2) from the set
   x = s.pop()

 Unique operations of PriorityQueue:
   PriorityQueue(order="min", f=lambda v: v): build up a priority queue
       using the function f to compute the priority based on the value
       of an element
   s.put(x, v): add the element x with value v into the queue
                update the value of x if x is already in the queue
   s.get(x): get the value of the element x
            raise KeyError if x is not in s
   s.pop(): return and remove the element with highest priority in s;
            raise IndexError if s is empty
            if order is "min", the element with minimum f(v) will be popped;
            if order is "max", the element with maximum f(v) will be popped.
 Example:
   s = PriorityQueue(order="min", f=lambda v: v.f)
   s.put((1,1,1), Value(f=2,g=1))
   s.put((2,2,2), Value(f=5,g=2))
   x, v = s.pop()  # the element with minimum value of v.f will be popped
"""

# ----------------------------------------
# modify the code below
# ----------------------------------------
def compute_path(grid,start,goal,cost,heuristic):
   
    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use thePriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)      

    # Keep track of the parent of each node. Since the car can take 4 distinct orientations, 
    # for each orientation we can store a 2D array indicating the grid cells. 
    # E.g. parent[0][2][3] will denote the parent when the car is at (2,3) facing up

    parent = [[['' for row in range(len(grid[0]))] for col in range(len(grid))],
              [['' for row in range(len(grid[0]))] for col in range(len(grid))],
              [['' for row in range(len(grid[0]))] for col in range(len(grid))],
              [['' for row in range(len(grid[0]))] for col in range(len(grid))]]

    # The path of the car
    path =[['-' for row in range(len(grid[0]))] for col in range(len(grid))]

    x = start[0]
    y = start[1]
    theta = start[2]
    path[x][y] = action_name[1]
    h = heuristic[x][y]
    g = 0
    f = g+h
    open_set.put(start, Value(f=f,g=g))
    closed_set.clear()
    # start from here

    for child in range(0,20): # A for loop to to run the children and update the open and closed sets.
        node = open_set.pop()  # Initially the open loop contains start and later on the min cost children gets updated.
        if node not in open_set or closed_set:
            closed_set.add(node) # Only if the node is not already present in the closed or open set, it adds.
        x = node[0][0] # Assigning the x coordinate of node to x variable.
        y = node[0][1] # Assigning the y coordinate of node to y variable.
        current_orient = node[0][2] # Assigning the current orientation of the node to a variable.
        orient_f = action[1] # assigning the action values to new variables for easy understanding
        orient_rf = action[0]
        orient_lf = action[2]
        if current_orient == 0: # Moving in North direction
            if x - 1 >= 0 and grid[x - 1][y] == 0: # Moving in forward direction
                theta = current_orient + orient_f # Adding the orientation value locally based on the global orientation
                node = (x - 1, y, theta)
                h = heuristic[x-1][y]
                g = cost[1]  # Using the cost funtion to the g value.
                f = g+ h
                open_set.put(node, Value(f=f, g=g)) # Adding the each node to the open set.
                path[x-1][y] = action_name[1]
                if x -1 == 0:
                    path[x-1][y] = action_name[0]

            if y-1 >=0 and grid[x][y-1] == 0: # Moving in left direction.
                theta = orient_lf + current_orient
                node = (x, y - 1, theta)
                # path[x][y-1] = 'L'
                h = heuristic[x][y-1]
                g = cost[2]
                f = g+h
                open_set.put(node, Value(f=f, g=g))

            if y + 1 < 5 and grid[x][y + 1] == 0: # Moving in right direction.
                if current_orient == 0:
                    orient_rf = 3
                theta = orient_rf + current_orient
                node = (x, y + 1, theta)
                # path[x][y + 1] = 'R'
                g = cost[0]
                h = heuristic[x][y+1]
                f = g+h
                open_set.put(node, Value(f=f, g=g))

        if current_orient == 1:  # Moving in West direction
            if x - 1 >= 0 and grid[x - 1][y] == 0: # Moving towards right.
                theta = current_orient + orient_rf
                node = (x - 1, y, theta)
                # path[x-1][y] = 'R'
                h = heuristic[x-1][y]
                g = cost[0]
                f = g+ h
                open_set.put(node, Value(f=f, g=g))

            if y-1 >=0 and grid[x][y-1] == 0: # Moving in forward direction
                theta = orient_f + current_orient
                node = (x, y - 1, theta)
                # path[x][y - 1] = 'F'
                h = heuristic[x][y-1]
                g = cost[1]
                f = g+h
                open_set.put(node, Value(f=f, g=g))
                path[x][y-1] = action_name[1]

            if x + 1 < 5 and grid[x+1][y] == 0: # Moving in left direction
                theta = orient_lf + current_orient
                node = (x +1, y, theta)
                # path[x+1][y] = 'L'
                h = heuristic[x+1][y]
                g = cost[2]
                f = g+h
                open_set.put(node, Value(f=f, g=g))
            # if node not in open_set or closed_set:
            #     closed_set.add(node)

        if current_orient == 3: # Moving in east direction
            if x + 1 < 5 and grid[x + 1][y] == 0: # Moving in right direction
                theta = current_orient + orient_rf
                current_orient = theta
                node = (x + 1, y, theta)
                h = heuristic[x+1][y]
                g = cost[0]
                f = g + h
            if x-1 >=0 and grid[x-1][y] == 0: # Moving in left direction
                theta = orient_lf + current_orient
                if theta ==4:
                    theta = 0
                current_orient = theta
                print('x', x)
                node = (x-1, y, theta)
                h = heuristic[x-1][y]
                g = cost[2]
                f = g+h
                open_set.put(node, Value(f=f, g=g))

            if y + 1 < 6 and grid[x][y + 1] == 0: # Moving in forward direction
                theta = orient_f + current_orient
                current_orient = theta
                node = (x, y + 1, theta)
                h = heuristic[x][y+1]
                g = cost[1]
                f = g+h
                open_set.put(node, Value(f=f, g=g))
                path[x][y+ 1] = action_name[0]
                path[x][y] = action_name[1]

        if current_orient == 2: # Moving in South direction.
            if x + 1 < 6 and grid[x + 1][y] == 0: # Moving in forward direction
                theta = orient_f + current_orient
                node = (x + 1, y, theta)
                h = heuristic[x + 1][y]
                g = cost[1]
                f = g + h
                open_set.put(node, Value(f=f, g=g))
                path[x][y] = action_name[1]

            if y + 1 < 6 and grid[x][y + 1] == 0: # Moving in left direction
                theta = orient_lf + current_orient
                print('theta', theta)
                node = (x, y + 1, theta)
                h = heuristic[x][y+1]
                g = cost[2]
                f = g+h
                open_set.put(node, Value(f=f, g=g))

            if y-1 >= 0 and grid[x][y-1] == 0: # Moving in right direction.
                theta = orient_rf + current_orient
                node = (x, y - 1, theta)
                h = heuristic[x][y-1]
                g = cost[0]
                f = g+h
                open_set.put(node, Value(f = f, g= g))
        if node == goal:  # If the node is currently at goal, then break the loop by adding that node to the closed set.
            open_set.put(node, Value(f=f, g=g))
            closed_set.add(node)
            path[2][0] = '*'
            break
    # your code: implement A*
    # Initially you may want to ignore theta, that is, plan in 2D.
    # To do so, set actions=forward, cost = [1, 1, 1, 1], and action_name = ['U', 'L', 'R', 'D']
    # Similarly, set parent=[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]

    return path, closed_set


if __name__ == "__main__":
    path,closed=compute_path(grid, init, goal, cost, heuristic)

    for i in range(len(path)):
        print(path[i])

    print("\nExpanded Nodes")    
    for node in closed:
        print(node)

"""
To test the correctness of your A* implementation, when using cost = [1, 1, 10] your code should return 

['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-'] 

In this case, the elements in your closed set (i.e. the expanded nodes) are: 
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

"""