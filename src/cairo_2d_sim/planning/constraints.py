
import numpy as np
from decimal import Decimal, localcontext, ROUND_DOWN

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

 
def project_config(tsr, q_s):
    return tsr.project(q_s)
    
 
class UnconstrainedTSR():
    
    def project(self, p):
        return p
 
class LineConstraintTSR():
    
    def __init__(self, p1, p2):
      
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        if np.sum((self.p1-self.p2)**2) == 0:
            raise Exception("p1 and p2 are the same points, no line exists")
        self.epislon_error = 10
        
    def project(self, p):
        M = np.array(self.p2[0:2]) - np.array(self.p1[0:2])
        t0 = np.dot(p[0:2] - self.p1[0:2], M) / np.dot(M, M);
        line_proj = self.p1[0:2] + np.dot(t0, M);
        projected_point = []
        if line_proj[0] < self.p1[0]:
            projected_point.append(self.p1[0])
        elif line_proj[0] > self.p2[0]:
            projected_point.append(self.p2[0])
        else:
            projected_point.append(line_proj[0])
        
        if line_proj[1] < self.p1[1]:
            projected_point.append(self.p1[1])
        elif line_proj[1] > self.p2[1]:
            projected_point.append(self.p2[1])
        else:
            projected_point.append(line_proj[1])
        projected_point.append(p[2])
        return projected_point