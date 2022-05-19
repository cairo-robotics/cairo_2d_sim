
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

 
def project_config(tsr, q_s, q_near, extension_distance):
    return tsr.project(q_s, q_near, extension_distance)
    
 
class UnconstrainedTSR():
    
    def project(self, p, q_near, extension_distance):
        v1 = p[0] - q_near[0]
        v2 = p[1] - q_near[1]
        v_norm = (v1**2 + v2**2)**.5
        ext_line_proj = [q_near[0] + extension_distance * v1/v_norm, q_near[1] + extension_distance * v2/v_norm, p[2]]  
        return ext_line_proj
 
class LineConstraintTSR():
    
    def __init__(self, p1, p2):
      
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        if np.sum((self.p1-self.p2)**2) == 0:
            raise Exception("p1 and p2 are the same points, no line exists")
        self.epislon_error = 10
        
    def project(self, p, q_near, extension_distance):
        M = np.array(self.p2[0:2]) - np.array(self.p1[0:2])
        t0 = np.dot(p[0:2] - self.p1[0:2], M) / np.dot(M, M);
        line_proj = self.p1[0:2] + np.dot(t0, M);
        v1 = line_proj[0] - q_near[0]
        v2 = line_proj[1] - q_near[1]
        v_norm = (v1**2 + v2**2)**.5
        x_extension = min(abs(q_near[0] - line_proj[0]), abs(extension_distance * v1/v_norm))
        y_extension = min(abs(q_near[1] - line_proj[1]), abs(extension_distance * v2/v_norm))
        if q_near[0] > line_proj[0]:
            x_extension = -x_extension
        if q_near[1] > line_proj[1]:
            y_extension = -y_extension
        ext_line_proj = [q_near[0] + x_extension, q_near[1] + y_extension, p[2]]
        projected_point = []
        if ext_line_proj[0] < self.p1[0]:
            projected_point.append(self.p1[0])
        elif ext_line_proj[0] > self.p2[0]:
            projected_point.append(self.p2[0])
        else:
            projected_point.append(ext_line_proj[0])
        
        if ext_line_proj[1] < self.p1[1]:
            projected_point.append(self.p1[1])
        elif ext_line_proj[1] > self.p2[1]:
            projected_point.append(self.p2[1])
        else:
            projected_point.append(ext_line_proj[1])
       
        if v_norm == 0:
            return None
        projected_point.append(p[2])
        return projected_point

class LineTargetingConstraintTSR():
    
    def __init__(self, p1, p2, target):
      
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.target = np.array(target)
        if np.sum((self.p1-self.p2)**2) == 0:
            raise Exception("p1 and p2 are the same points, no line exists")
        
    def project(self, p, q_near, extension_distance):
       line_projection = self._line_projection(p, q_near, extension_distance)
       if line_projection is None:
           return None
       theta = self._theta_projection(line_projection)
       line_projection.append(theta)
       return line_projection
    
    def _line_projection(self, p, q_near, extension_distance):
        M = np.array(self.p2[0:2]) - np.array(self.p1[0:2])
        t0 = np.dot(p[0:2] - self.p1[0:2], M) / np.dot(M, M);
        line_proj = self.p1[0:2] + np.dot(t0, M);
        v1 = line_proj[0] - q_near[0]
        v2 = line_proj[1] - q_near[1]
        v_norm = (v1**2 + v2**2)**.5
        ext_line_proj = [q_near[0] + extension_distance * v1/v_norm, q_near[1] + extension_distance * v2/v_norm, p[2]]
        projected_point = []
        if ext_line_proj[0] < self.p1[0]:
            projected_point.append(self.p1[0])
        elif ext_line_proj[0] > self.p2[0]:
            projected_point.append(self.p2[0])
        else:
            projected_point.append(ext_line_proj[0])
        
        if ext_line_proj[1] < self.p1[1]:
            projected_point.append(self.p1[1])
        elif ext_line_proj[1] > self.p2[1]:
            projected_point.append(self.p2[1])
        else:
            projected_point.append(ext_line_proj[1])
       
        if v_norm == 0:
            return None
        return projected_point
    
    def _theta_projection(self, p):
        return 360 - atan2(self.target[0] - p[1], self.target[1] - p[0]) * 180 / pi