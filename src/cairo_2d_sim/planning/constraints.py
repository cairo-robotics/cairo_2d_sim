
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
        point = np.array(p)
        ap = self.p1 - point
        ab = self.p2 - point
        projected_point = point + np.dot(ap, ab) / np.dot(ab, ab) * ab
        return projected_point