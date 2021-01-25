# simulator.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import numpy as np
from math import sqrt
import math

class Agent(object):

    def __init__(self, csvParameters, ksi=0.5, dhor = 10, timehor=5, goalRadiusSq=1, maxF = 10):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq = goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.dhor = dhor # the sensing radius
        self.timehor = timehor # the time horizon for computing avoidance forces
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent

    def computeForces(self, neighbors=[]):
        """ 
            Your code to compute the forces acting on the agent. 
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors
        """

        distance_rela = []
        v_rel = []
        epsilon = 0.2
	k = 2
        x_list =[]
        for agent in neighbors:
            if not self.id == agent.id:
                x_cord = (self.pos[0] - self.goal[0]) ** 2
                y_cord = (self.pos[1] - self.goal[1]) ** 2
                xy_dist = sqrt(x_cord + y_cord)
                x_list.append(xy_dist)  # Creating a list of distances.
                xmin = min(x_list)  # Finding the minimum of the list.
                sensing_rad = sqrt(np.dot(self.pos - agent.pos, self.pos - agent.pos))
                if xmin < 3 and self.gid == agent.gid:
                    sensing_rad = 10
                    self.F = 0.02
                else:
                    if sensing_rad < 7:
                        v = self.vel - agent.vel
                        v_rel.append(v)
                        distance_rel = self.pos - agent.pos
                        distance_rela.append(distance_rel)
        power_total = [0,0]
        f_sum = [0,0]
        power_force =[0,0]
        goal_force = 2 * (self.gvel - self.vel)
        for var in range(len(v_rel)):
            dist = distance_rela[var]
            v_rela = v_rel[var]
            a = np.dot(v_rela, v_rela) - (epsilon) ** 2
            b = np.dot(dist, v_rela) - epsilon * self.radius  # Calculating other values and computing time.
            c = np.dot(dist, dist) - (self.radius + self.radius)**2
            d = b * b - a * c
            if d <= 0:
                tau = math.inf
            else:
                d = sqrt(d)
                tau = (-b-d)/a # Least time of collision.
                if tau < 0:
                    tau = math.inf  # No collision = no force
                else:
                    power_f = dist + v_rela*tau
                    power_force = 2*k*(power_f / d)/tau**3
                    power_total += power_force
        f_sum = goal_force + power_total
        if f_sum[0] > 10:
            f_sum[0] = 10
        if f_sum[0] < - 10:
            f_sum[0] = - 10
        if f_sum[1] > 10:
            f_sum[1] = 10
        if f_sum[1] < - 10:
            f_sum[1] = - 10
        self.F = f_sum
    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel += self.F * dt  # update the velocity
            self.pos += self.vel * dt  # update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed  
            
            
  