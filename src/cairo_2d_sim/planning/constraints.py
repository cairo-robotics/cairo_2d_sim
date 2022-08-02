
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
    
    def __init__(self, p1, p2, bounds=(5,)):
      
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.bounds = bounds
        if np.sum((self.p1-self.p2)**2) == 0:
            raise Exception("p1 and p2 are the same points, no line exists")
    
    def distance(self, p):
        if np.shape(p) != (2,):
            p = p[0:2]
        line_proj = self._line_projection(p)
        return np.linalg.norm(line_proj - p)
    
    def validate(self, p):
        line_proj = self._line_projection(p)
        if np.linalg.norm(line_proj - p[0:2]) < self.bounds[0]:
            return True
        else:
            return False

    def project(self, p, _):
        line_proj = self._line_projection(p)
        projected_point = []
        projected_point.append(line_proj[0])
        projected_point.append(line_proj[1])
        projected_point.append(p[2])
        return projected_point
    
    def _line_projection(self, p):
        M = np.array(self.p2[0:2]) - np.array(self.p1[0:2])
        t0 = np.dot(p[0:2] - self.p1[0:2], M) / np.dot(M, M)
        line_proj = self.p1[0:2] + np.dot(t0, M)
        
        # endpoint accomodation
        x_values = sorted([self.p1[0], self.p2[0]])
        y_values = sorted([self.p1[1], self.p2[1]])
        
        if line_proj[0] < x_values[0]:
            line_proj[0] = x_values[0]
        elif line_proj[0] > x_values[1]:
            line_proj[0] = x_values[1]
        
        if line_proj[1] < y_values[0]:
            line_proj[1] = y_values[0]
        elif line_proj[1] > y_values[1]:
            line_proj[1] = y_values[1]
        
        return line_proj

class LineTargetingTSR():
    
    def __init__(self, p1, p2, target, bounds=(5, 5)):
        self.p1 = np.array(p1)
        self.p2 = np.array(p2)
        self.target = np.array(target)
        self.bounds = bounds
        if np.sum((self.p1-self.p2)**2) == 0:
            raise Exception("p1 and p2 are the same points, no line exists")
    
    def distance(self, p):
        line_proj = self._line_projection(p)
        theta_p = self._theta_projection(p)
        theta_line_proj = self._theta_projection(line_proj)
        angle_diff = theta_p - theta_line_proj
        theta_delta = abs((angle_diff + 180) % 360 - 180)
        xy_dist = np.linalg.norm(line_proj - p[0:2])
        bound_vec = self._distance_to_bounds(xy_dist, theta_delta)
        return np.linalg.norm(bound_vec)
    
    def validate(self, p):
        line_proj = self._line_projection(p)
        theta_p = self._theta_projection(p)
        theta_line_proj = self._theta_projection(line_proj)
        angle_diff = theta_p - theta_line_proj
        theta_delta = abs((angle_diff + 180) % 360 - 180)
        xy_dist = np.linalg.norm(line_proj - p[0:2])
        bounds_vec = self._distance_to_bounds(xy_dist, theta_delta)
        if all([value == 0 for value in bounds_vec]):
            return True
        else:
            return False
    
    def project(self, p, _):
        line_proj = self._line_projection(p)
        if line_proj is None:
           return None
        theta = self._theta_projection(line_proj)
        line_proj.append(theta)
        return line_proj
    
    def _line_projection(self, p):
        M = np.array(self.p2[0:2]) - np.array(self.p1[0:2])
        t0 = np.dot(p[0:2] - self.p1[0:2], M) / np.dot(M, M)
        line_proj = self.p1[0:2] + np.dot(t0, M)
        
        # endpoint accomodation
        x_values = sorted([self.p1[0], self.p2[0]])
        y_values = sorted([self.p1[1], self.p2[1]])
        
        if line_proj[0] < x_values[0]:
            line_proj[0] = x_values[0]
        elif line_proj[0] > x_values[1]:
            line_proj[0] = x_values[1]
        
        if line_proj[1] < y_values[0]:
            line_proj[1] = y_values[0]
        elif line_proj[1] > y_values[1]:
            line_proj[1] = y_values[1]

        projected_point = []
        projected_point.append(line_proj[0])
        projected_point.append(line_proj[1])
        
        return projected_point
    
    def _theta_projection(self, p):
        return 360 - atan2(self.target[1] - p[1], self.target[0] - p[0]) * 180 / pi

    def _distance_to_bounds(self, xy_dist, angle_diff):
        d1 = 0
        if abs(xy_dist) < self.bounds[0]:
            d1 = 0
        else:
            d1 = abs(xy_dist) - self.bounds[0]
        d2 = 0
        if abs(angle_diff) < self.bounds[1]:
            d2 = 0
        else:
            d2 = abs(angle_diff) - self.bounds[1]
        return [d1, d2]
        

class DualLineTargetingTSR():
    
    def __init__(self, l1p1, l1p2, l2p1, l2p2, target, bounds=(5, 5)):
      
        self.l1p1 = np.array(l1p1)
        self.l1p2 = np.array(l1p2)
        self.l2p1 = np.array(l2p1)
        self.l2p2 = np.array(l2p2)
        self.target = np.array(target)
        self.bounds = bounds
        if np.sum((self.l1p1-self.l1p2)**2) == 0 or np.sum((self.l2p1-self.l2p2)**2) == 0:
            raise Exception("p1 and p2 are the same points, no line exists")
    
    def validate(self, p):
        line_proj = self._line_projection(p, None)
        distance_to_line = np.linalg.norm(np.array(line_proj) - p[0:2])
        theta_diff = p[2] - self._theta_projection(line_proj)
        theta_delta = abs((theta_diff + 180) % 360 - 180)
        bounds_vec = self._distance_to_bounds(distance_to_line, theta_delta)
        if all([value == 0 for value in bounds_vec]):
            return True
        else:
            return False
    
    def distance(self, p):
        line_proj = self._line_projection(p, None)
        theta = 360 - atan2(self.target[1] - p[1], self.target[0] - p[0]) * 180 / pi
        angle_diff = p[2] - theta
        xy_dist = np.linalg.norm(np.array(line_proj[0:2]) - np.array(p[0:2]))
        bounds = self._distance_to_bounds(xy_dist, angle_diff)
        return np.linalg.norm(bounds)
    
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
        line_2_proj = self.l2p1[0:2] + np.dot(t20, M2)
        
        if q_near is not None:
            if np.linalg.norm(line_1_proj - q_near[0:2]) < np.linalg.norm(line_2_proj - q_near[0:2]):
                ref_p1 = self.l1p1
                ref_p2 = self.l1p2
                line_proj = line_1_proj
            else:
                ref_p1 = self.l2p1
                ref_p2 = self.l2p2
                line_proj = line_2_proj
        else:
            if np.linalg.norm(line_1_proj - p[0:2]) < np.linalg.norm(line_2_proj - p[0:2]):
                ref_p1 = self.l1p1
                ref_p2 = self.l1p2
                line_proj = line_1_proj
            else:
                ref_p1 = self.l2p1
                ref_p2 = self.l2p2
                line_proj = line_2_proj
        # endpoint accomodation
        x_values = sorted([ref_p1[0], ref_p2[0]])
        y_values = sorted([ref_p1[1], ref_p2[1]])
        if line_proj[0] < x_values[0]:
            line_proj[0] = x_values[0]
        elif line_proj[0] > x_values[1]:
            line_proj[0] = x_values[1]
        if line_proj[1] < y_values[0]:
            line_proj[1] = y_values[0]
        elif line_proj[1] > y_values[1]:
            line_proj[1] = y_values[1]
        
        return list(line_proj)
    
    def _theta_projection(self, p):
        return 360 - atan2(self.target[1] - p[1], self.target[0] - p[0]) * 180 / pi

    def _distance_to_bounds(self, xy_dist, angle_diff):
        d1 = 0
        if abs(xy_dist) < self.bounds[0]:
            d1 = 0
        else:
            d1 = abs(xy_dist) - self.bounds[0]
        d2 = 0
        if abs(angle_diff) < self.bounds[1]:
            d2 = 0
        else:
            d2 = abs(angle_diff) - self.bounds[1]
        return [d1, d2]