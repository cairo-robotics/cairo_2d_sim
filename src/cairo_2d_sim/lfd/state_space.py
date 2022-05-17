import random


class StateValidityChecker():
    
    def validate(self, q):
        return True

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