
from math import pi, atan2
from decimal import Decimal, localcontext, ROUND_DOWN

import numpy as np

def name2idx(igraph, name):
    try:
        return igraph.vs.find(name).index
    except Exception as e:
        pass

def val2str(value, decimal_places=8):
    def trunc(number, places=decimal_places):
        if not isinstance(places, int):
            raise ValueError("Decimal places must be an integer.")
        if places < 1:
            raise ValueError("Decimal places must be at least 1.")
        # If you want to truncate to 0 decimal places, just do int(number).

        with localcontext() as context:
            context.rounding = ROUND_DOWN
            exponent = Decimal(str(10 ** - places))
            return Decimal(str(number)).quantize(exponent).to_eng_string()
    return str([trunc(num, decimal_places) for num in value])

 
def project_config(tsr, q_s, q_near):
    projected_point = tsr.project(q_s, q_near)
    return projected_point

    
class UnconstrainedTSR():
    
    def validate(self, _):
        return True
    
    def distance(self, p):
        return 0.0
    
    def project(self, p, _):
        return p
 
class LineTSR():
    
    def __init__(self, p1, p2):
      
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        if np.sum((self.p1-self.p2)**2) == 0:
            raise Exception("p1 and p2 are the same points, no line exists")
    
    def distance(self, p):
        M = np.array(self.p2[0:2]) - np.array(self.p1[0:2])
        t0 = np.dot(p[0:2] - self.p1[0:2], M) / np.dot(M, M)
        line_proj = self.p1[0:2] + np.dot(t0, M)
        return np.linalg.norm(line_proj - p)
    
    def validate(self, p):
        M = np.array(self.p2[0:2]) - np.array(self.p1[0:2])
        t0 = np.dot(p[0:2] - self.p1[0:2], M) / np.dot(M, M)
        line_proj = self.p1[0:2] + np.dot(t0, M)
        if np.linalg.norm(line_proj - p) < 5:
            return True
        else:
            return False

    def project(self, p, _):
        M = np.array(self.p2[0:2]) - np.array(self.p1[0:2])
        t0 = np.dot(p[0:2] - self.p1[0:2], M) / np.dot(M, M)
        line_proj = self.p1[0:2] + np.dot(t0, M)
        
        projected_point = []
        projected_point.append(line_proj[0])
        projected_point.append(line_proj[1])
        projected_point.append(p[2])

        return projected_point

class LineTargetingTSR():
    
    def __init__(self, p1, p2, target):
      
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.target = np.array(target)
        if np.sum((self.p1-self.p2)**2) == 0:
            raise Exception("p1 and p2 are the same points, no line exists")
    
    def distance(self, p):
        M = np.array(self.p2[0:2]) - np.array(self.p1[0:2])
        t0 = np.dot(p[0:2] - self.p1[0:2], M) / np.dot(M, M)
        line_proj = self.p1[0:2] + np.dot(t0, M)
        return np.linalg.norm(line_proj - p)
    
    def validate(self, p):
        M = np.array(self.p2[0:2]) - np.array(self.p1[0:2])
        t0 = np.dot(p[0:2] - self.p1[0:2], M) / np.dot(M, M)
        line_proj = self.p1[0:2] + np.dot(t0, M)
        
        if np.linalg.norm(line_proj - p) < 5:
            return True
        else:
            return False
    
    def project(self, p, _):
       line_projection = self._line_projection(p)
       if line_projection is None:
           return None
       theta = self._theta_projection(line_projection)
       line_projection.append(theta)
       return line_projection
    
    def _line_projection(self, p):
        M = np.array(self.p2[0:2]) - np.array(self.p1[0:2])
        t0 = np.dot(p[0:2] - self.p1[0:2], M) / np.dot(M, M);
        line_proj = self.p1[0:2] + np.dot(t0, M);
        
        projected_point = []
        projected_point.append(line_proj[0])
        projected_point.append(line_proj[1])
        
        return projected_point
    
    def _theta_projection(self, p):
        return 360 - atan2(self.target[1] - p[1], self.target[0] - p[0]) * 180 / pi

class DualLineTargetingTSR():
    
    def __init__(self, l1p1, l1p2, l2p1, l2p2, target):
      
        self.l1p1 = np.array(l1p1)
        self.l1p2 = np.array(l1p2)
        self.l2p1 = np.array(l2p1)
        self.l2p2 = np.array(l2p2)
        self.target = np.array(target)
        if np.sum((self.l1p1-self.l1p2)**2) == 0 or np.sum((self.l2p1-self.l2p2)**2) == 0:
            raise Exception("p1 and p2 are the same points, no line exists")
    
    def validate(self, p):
        M1 = np.array(self.l1p2[0:2]) - np.array(self.l1p1[0:2])
        t10 = np.dot(p[0:2] - self.l1p1[0:2], M1) / np.dot(M1, M1)
        line_1_proj = self.l1p1[0:2] + np.dot(t10, M1)
        
        M2 = np.array(self.l2p2[0:2]) - np.array(self.l2p1[0:2])
        t20 = np.dot(p[0:2] - self.l2p1[0:2], M2) / np.dot(M2, M2)
        line_2_proj = self.l1p1[0:2] + np.dot(t20, M2)
        
        if np.linalg.norm(line_1_proj - p) < 5 or np.linalg.norm(line_2_proj - p) < 5:
            return True
        else:
            return False
    
    def project(self, p, q_near):
       line_projection = self._line_projection(p, q_near)
       if line_projection is None:
           return None
       theta = self._theta_projection(line_projection)
       line_projection.append(theta)
       return line_projection
    
    def _line_projection(self, p, q_near):
        M1 = np.array(self.l1p2[0:2]) - np.array(self.l1p1[0:2])
        t10 = np.dot(p[0:2] - self.l1p1[0:2], M1) / np.dot(M1, M1)
        line_1_proj = self.l1p1[0:2] + np.dot(t10, M1)
        
        M2 = np.array(self.l2p2[0:2]) - np.array(self.l2p1[0:2])
        t20 = np.dot(p[0:2] - self.l2p1[0:2], M2) / np.dot(M2, M2)
        line_2_proj = self.l1p1[0:2] + np.dot(t20, M2)
        
        if q_near is not None:
            if np.linalg.norm(line_1_proj - q_near[0:2]) < np.linalg.norm(line_2_proj - q_near[0:2]):
                line_proj = line_1_proj
            else:
                line_proj = line_2_proj
        else:
            if np.linalg.norm(line_1_proj - p[0:2]) < np.linalg.norm(line_2_proj - p[0:2]):
                line_proj = line_1_proj
            else:
                line_proj = line_2_proj
        
        return list(line_proj)
    
    def _theta_projection(self, p):
        return 360 - atan2(self.target[1] - p[1], self.target[0] - p[0]) * 180 / pi