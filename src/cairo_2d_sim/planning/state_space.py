import random

import numpy as np

from cairo_2d_sim.planning.sampling import DistributionSampler

class StateValidityChecker():
    
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal  
        self.epislon = 5
    
    def validate(self, q):
        # if self._close_to_goal(q) or self._close_to_start(q):
        #     return False
        return True

    def _close_to_start(self, q):
        if np.linalg.norm(q[0:2] - self.start[0:2]) <= self.epislon:
            return True
        return False
    
    def _close_to_goal(self, q):
        if np.linalg.norm(q[0:2] - self.goal[0:2]) <= self.epislon:
            return True
        return False
    
class Holonomic2DStateSpace():
    
    def __init__(self, x_domain, y_domain, theta_domain=(0, 360)):
        self.x_domain = x_domain
        self.y_domain = y_domain
        self.theta_domain = theta_domain
            
    def sample(self):
        x_rand = random.uniform(self.x_domain[0], self.x_domain[1])
        y_rand = random.uniform(self.y_domain[0], self.y_domain[1])
        theta_rand = random.uniform(self.theta_domain[0], self.theta_domain[1])
        
        return [x_rand, y_rand, theta_rand]
    

class Holonomic2DBiasedStateSpace():
    
    def __init__(self, distribution_model, x_domain, y_domain, theta_domain=(0, 360), ):
        self.x_domain = x_domain
        self.y_domain = y_domain
        self.theta_domain = theta_domain
        self.sampler = DistributionSampler(distribution_model)
            
    def sample(self):
        return self.sampler.sample([self.x_domain, self.y_domain, self.theta_domain])
    
    
