
import numpy as np
 
def val2str(q_val):
    pass

def name2idx(B, qb_name):
    pass

 
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