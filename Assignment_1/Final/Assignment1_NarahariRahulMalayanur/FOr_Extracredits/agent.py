# agent.py
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
import random
class Agent(object):
    def __init__(self, csvParameters, dhor = 10, goalRadiusSq=1):
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
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.dhor = dhor # the sensing radius
        self.vnew = np.zeros(2) # the new velocity of the agent



    def computeNewVelocity(self, neighbors=[]):
        """ 
            Your code to compute the new velocity of the agent. 
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors.
            The code should set the vnew of the agent to be one of the sampled admissible one. Please do not directly set here the agent's velocity.   
        """

        # Setting the parameters for computing the cost function.
        alpha = 3
        beta = 5
        gama = 2
        tau = []          # Creating null lists for appending the data values.
        distancea = []
        AV = np.zeros((200, 2))
        relative_v =[]
        xmap = []
        ymap = []
        cf = []
        nlist =[]
        x_list =[]

        for k in neighbors:
            sensing_rad = sqrt(np.dot(self.pos - k.pos, self.pos - k.pos))
            if sensing_rad < self.dhor:
                nlist.append(k)
        for agent in nlist:
            if not agent.id == self.id:  # it into another lists for later use.
                distance = self.pos - agent.pos
                distancea.append(distance)
                relative_v.append(agent.vel)
                x = sqrt(np.dot(agent.pos - agent.goal, agent.pos - agent.goal)) # Distance b/w current position and the goal position
                x_list.append(x) # Creating a list of distances.
                xmin = min(x_list) # Finding the minimum of the list.


        for i in range(0, 200):  # By using the random.uniform function, generating random velocities
            r = random.uniform(0, 4)  # inside its maximum velocity and appending those values in a list.
            t = random.uniform(0, 2 * math.pi)
            xmap = sqrt(r) * math.cos(t)
            ymap = sqrt(r) * math.sin(t)
            AV[i][0] = xmap
            AV[i][1] = ymap
            vel = (AV[i][0], AV[i][1])
            vel_mat = []
            vel_mat.append(vel)  # Appending the values.

        rad = 2 * self.radius
        cf_list = []
        for var in range(len(AV)):  # Now calculating the time for each set of candidate velocities.
            time_mat =[]
            for var1 in range(len(relative_v)):  # Now calculating distances and relative velocities between the
                v = AV[var] - relative_v[var1]  # agent velocities and the candidate velocities.
                distance1 = distancea[var1]
                distdot = np.dot(distance1, distance1)

                c = distdot - rad * rad
                if c < 0 and (xmin < 3 and self.gid == self.gid): # Making sure the same gid agents do not collide when they
                    time = 10^8
                elif c < 0 :  # Calculating the time of collision using the if else statements and
                    time = 0  # To break the loop from running when the agents collide.
                    print("You have collided with other Agent, try again!")
                else:
                    b = np.dot(distance1, v) # Calculating other values and computing time.
                    a = np.dot(v, v)
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
                            elif tau >= 0:
                                time = tau

                time_mat.append(time)  # Appending the time values in a matrix.
            timemin = min(time_mat) # Finding the minimum time taken to collide.
            alpha_term = AV[var] - self.gvel  # Cost function calculations
            alpha_dot = np.dot(alpha_term, alpha_term)
            beta_term = AV[var] - self.vel
            beta_dot = np.dot(beta_term, beta_term)
            cf = alpha * sqrt(alpha_dot) + beta * sqrt(beta_dot) + (gama * (1 / timemin))

            cf_list.append(cf)
        min_cf = min(cf_list)

        value = cf_list.index(min_cf)
        self.vnew[:] = AV[value]

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agent  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed  

  