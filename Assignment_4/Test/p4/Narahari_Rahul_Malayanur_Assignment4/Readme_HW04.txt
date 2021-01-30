CPSC 8810- MOTION PLANNING- SPRING 2020:

HOMEWORK 4: PRM BASED PLANNING

INSTRUCTOR: IOANNIS KARAMOUZAS

TEACHING ASSISTANT: PEI XU

ASSIGNMENT TEAMMATES: ADITHYA SURESH and NARAHARI RAHUL MALAYANUR

SUBMISSION DATE: 03/10/2020

ASSIGNMENT WORK DETAILS:

LIBRARIES USED:

import numpy as np
from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
from scene import *
import random
from math import sqrt
import math

The task is to build a roadmap that captures the connectivity of free configuration space. With the task involved, we have divided the work into the construction phase and a query phase.

CONSTRUCTION PHASE:

1) In the construction phase, initially, we are adding the vertices on the C-obstacle free space with the help of collision function. 

2) After adding the vertice, edges are defined between the vertices which are less than a maximum distance of 5. Once the edges are added, the interpolate function is called to check whether the edges pass-through 
   the obstacles or not. Also, the unnecessary edges are removed, which if present increases computation time.

3) We have used three functions in the prmplanner file:
 1) Distance function : This was used to calculate the distance between any two vertices.
 2) Collision function: This was used to check whether the given vertex is present in the collision space or not. 
 3) Interpolate function: This was used to check if the edge between the vertices is passing over the collision obstacle or not. The following website was referred to arrive to a formula, which generally checks
    if any two line segment is joining or not. 
    http://www.cs.swan.ac.uk/~cssimon/line_intersection.html   

QUERY PHASE:

The query phase's task is to form a path with the created vertices and edges using search algorithms like A*.

A* algorithm:

1) As done in the third assignment, we approached the A star with start and end goal values as inputs and calculated the distance calculations for the g and h (costs). 

2) A parent list is created for the movement from the current node to the subsequent nodes and the cost list is also created in the same method with the total range involving the length of the children present.

3) The children are appended concerning the ids of edges between the current and the neighboring nodes.

4) Finally, the optimal path is predicted using A* algorithm by backtracking the parent to the goal point.

