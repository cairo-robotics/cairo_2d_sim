import random


class StateValidityChecker():
    
    def __init__(self, start, goal):
        self.start = start
        self.goal= goal  
    
    def validate(self, q):
        if self._equal_to_goal(q) or self._equal_to_start(q):
            return False
        return True

    def _equal_to_start(self, q):
        if self.start[0] == q[0] and self.start[1] == q[1]:
            return True
        return False
    
    def _equal_to_goal(self, q):
        if self.goal[0] == q[0] and self.goal[1] == q[1]:
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