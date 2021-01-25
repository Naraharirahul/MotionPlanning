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
        f_total = [0, 0]
        epsilon = 0
        x_list = []
        goal_force = 2 * (self.gvel - self.vel)
        for agent in neighbors:
            if not self.id == agent.id:
                sensing_rad = sqrt(np.dot(self.pos - agent.pos, self.pos - agent.pos))
                if sensing_rad < 7:
                    v = self.vel - agent.vel
                    v_rel.append(v)
                    distance_rel = self.pos - agent.pos
                    distance_rela.append(distance_rel)
                    x = sqrt(np.dot(agent.pos - agent.goal,agent.pos - agent.goal))  # Distance b/w current position and the goal position
                    x_list.append(x)  # Creating a list of distances.
                    xmin = min(x_list)  # Finding the minimum of the list.
                    if xmin < 3 and self.gid == agent.gid:
                        # self.vel = self.gvel
                        self.F = 0.2
                else:
                    self.F = goal_force
        for var in range(len(v_rel)):
            dist = distance_rela[var]
            v_rela =v_rel[var]
            c = np.dot(dist, dist) - (self.radius + self.radius)**2
            if c < 0:  # Calculating the time of collision using the if else statements and
                time = 0  # To break the loop from running when the agents collide.
                f_mag = 0
            else:
                a = np.dot(v_rela, v_rela) - (epsilon)**2
                b = np.dot(dist, v_rela) - epsilon*self.radius  # Calculating other values and computing time.
                if b > 0:
                    time = math.inf
                else:
                    d = b * b - a * c
                    if d <= 0:
                        time = math.inf
                    else:
                        tau = c / (-b + sqrt(d)) # Least time of collision.
                        if tau < 0:
                            time = math.inf
                        else:
                            time = tau
                            f_avoid = dist + (v_rela)*time
                            n_force = f_avoid / sqrt(np.dot(f_avoid, f_avoid))
                            f_mag = max(self.timehor - time, 0)/time
                            f_total += f_mag * n_force
        f_sum = goal_force + f_total
        if f_sum[0] > 7:
            f_sum[0] = 7
        if f_sum[0] < - 7:
            f_sum[0] = - 7
        if f_sum[1] > 7:
            f_sum[1] = 7
        if f_sum[0] < - 7:
            f_sum[1] = - 7
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
            
            
  