from itertools import tee
from math import pi, atan2

from gekko import GEKKO


class DualIntersectionOptimization():

    def __init__(self, first_intersection, second_intersection, height=1000, width=1800):
        self.first_intersection = first_intersection
        self.second_intersection = second_intersection
        self.height = height
        self.width = width
        self.epislon_error = 25
        self.model = None

    
    def solve(self, keyframe_point):
        m = GEKKO() # Initialize gekko
        m.options.SOLVER=1  # APOPT is an MINLP solver. See https://apopt.com/download.php

        # optional solver settings with APOPT
        m.solver_options = ['minlp_maximum_iterations 50000', \
                    # minlp iterations with integer solution
                    'minlp_max_iter_with_int_sol 1000', \
                    # treat minlp as nlp
                    'minlp_as_nlp 1', \
                    # nlp sub-problem max iterations
                    'nlp_maximum_iterations 5000', \
                    # 1 = depth first, 2 = breadth first
                    'minlp_branch_method 2', \
                    # maximum deviation from whole number
                    'minlp_integer_tol 0.1', \
                    # covergence tolerance
                    'minlp_gap_tol 0.01']

        # Initialize variables
        X = m.Var(value=keyframe_point[0])
        Y = m.Var(value=keyframe_point[1])
        # Integer variables.
        A = m.Var(lb=0, ub=1, integer=True)
     
        # Distance equations

        distance_from_first_intersection = A * ((X - self.first_intersection[0])**2 +
             (Y - self.first_intersection[1])**2)**0.5

        distance_from_second_intersection = (
            1-A) * ((X - self.second_intersection[0])**2 + (Y - self.second_intersection[1])**2)**0.5

        distance_from_intersection_combined = distance_from_first_intersection + \
            distance_from_second_intersection
            
        distance_from_keyframe_point = (
            (X - keyframe_point[0])**2 + (Y - keyframe_point[1])**2)**0.5
        
        m.Equation(X <= self.width)
        m.Equation(X >= 0)
        m.Equation(Y <= self.height)
        m.Equation(Y >= 0)
        m.Equation(A * distance_from_first_intersection <= self.epislon_error)
        m.Equation((1 - A) * distance_from_second_intersection <= self.epislon_error)
        
        m.Obj(distance_from_intersection_combined + distance_from_keyframe_point)
        self.model = m 
        try:
            self.model.solve(disp=False, debug=True)
            print('Results')
            print('A: ' + str(X.value))
            print('Y: ' + str(Y.value))
            print('A: ' + str(A.value))
            print('Objective: ' + str(m.options.objfcnval))
            self.model.cleanup()
            return [X.value[0], Y.value[0]]
        except Exception as e:
            print(e)
            self.model.cleanup()
            return None

class DualIntersectionWithTargetingOptimization():

    def __init__(self, first_intersection, second_intersection, target, height=1000, width=1800):
        self.first_intersection = first_intersection
        self.second_intersection = second_intersection
        self.target = target
        self.height = height
        self.width = width
        self.epislon_error = 25
        self.model = None

    
    def solve(self, keyframe_point):
        m = GEKKO() # Initialize gekko
        m.options.SOLVER=1  # APOPT is an MINLP solver. See https://apopt.com/download.php

        # optional solver settings with APOPT
        m.solver_options = ['minlp_maximum_iterations 50000', \
                    # minlp iterations with integer solution
                    'minlp_max_iter_with_int_sol 1000', \
                    # treat minlp as nlp
                    'minlp_as_nlp 1', \
                    # nlp sub-problem max iterations
                    'nlp_maximum_iterations 5000', \
                    # 1 = depth first, 2 = breadth first
                    'minlp_branch_method 2', \
                    # maximum deviation from whole number
                    'minlp_integer_tol 0.1', \
                    # covergence tolerance
                    'minlp_gap_tol 0.01']

        # Initialize variables
        X = m.Var(value=keyframe_point[0])
        Y = m.Var(value=keyframe_point[1])
        T = m.Var(value=keyframe_point[2])
        # Integer variables.
        A = m.Var(lb=0, ub=1, integer=True)
     
        # Distance equations

        distance_from_first_intersection = A * ((X - self.first_intersection[0])**2 +
             (Y - self.first_intersection[1])**2)**0.5

        distance_from_second_intersection = (
            1-A) * ((X - self.second_intersection[0])**2 + (Y - self.second_intersection[1])**2)**0.5

        distance_from_intersection_combined = distance_from_first_intersection + \
            distance_from_second_intersection
            
        distance_from_keyframe_point = (
            (X - keyframe_point[0])**2 + (Y - keyframe_point[1])**2)**0.5
                
        m.Equation(X <= self.width)
        m.Equation(X >= 0)
        m.Equation(Y <= self.height)
        m.Equation(Y >= 0)
        m.Equation(T == (360 - m.atan((self.target[1] - Y) / (self.target[0] - X)) * 180 / pi))
        m.Equation(A * distance_from_first_intersection <= self.epislon_error)
        m.Equation((1 - A) * distance_from_second_intersection <= self.epislon_error)
        
        m.Obj(distance_from_intersection_combined + distance_from_keyframe_point)
        self.model = m 
        try:
            self.model.solve(disp=False, debug=True)
            theta_actual = 360 - atan2(self.target[1] - Y.value[0], self.target[0] - X.value[0]) * 180 / pi
            print('Results')
            print('X: ' + str(X.value))
            print('Y: ' + str(Y.value))
            print('T: ' + str(T.value))
            print('A: ' + str(A.value))
            print('Objective: ' + str(m.options.objfcnval))
            self.model.cleanup()
            return [X.value[0], Y.value[0], T.value[0] % 360]
        except Exception as e:
            print(e)
            self.model.cleanup()
            return None

class SingleIntersectionWithTargetingOptimization():

    def __init__(self, intersection, target, height=1000, width=1800):
        self.intersection = intersection
        self.height = height
        self.width = width
        self.target = target
        self.epislon_error = 25
        self.model = None

    
    def solve(self, keyframe_point):
        m = GEKKO() # Initialize gekko
        m.options.SOLVER=1  # APOPT is an MINLP solver. See https://apopt.com/download.php

        # optional solver settings with APOPT
        m.solver_options = ['minlp_maximum_iterations 50000', \
                    # minlp iterations with integer solution
                    'minlp_max_iter_with_int_sol 1000', \
                    # treat minlp as nlp
                    'minlp_as_nlp 1', \
                    # nlp sub-problem max iterations
                    'nlp_maximum_iterations 5000', \
                    # 1 = depth first, 2 = breadth first
                    'minlp_branch_method 2', \
                    # maximum deviation from whole number
                    'minlp_integer_tol 0.1', \
                    # covergence tolerance
                    'minlp_gap_tol 0.01']

        # Initialize variables
        X = m.Var(value=keyframe_point[0])
        Y = m.Var(value=keyframe_point[1])
        T = m.Var(value=keyframe_point[2])

        # Distance equations

        distance_from_intersection = ((X - self.intersection[0])**2 +
             (Y - self.intersection[1])**2)**0.5

        distance_from_keyframe_point = (
            (X - keyframe_point[0])**2 + (Y - keyframe_point[1])**2)**0.5
        
        
        m.Equation(X <= self.width)
        m.Equation(X >= 0)
        m.Equation(Y <= self.height)
        m.Equation(Y >= 0)
        m.Equation(T == (360 - m.atan((self.target[1] - Y) / (self.target[0] - X)) * 180 / pi))
        m.Equation(distance_from_intersection <= self.epislon_error)
        
        m.Obj(3 * distance_from_intersection + distance_from_keyframe_point)
        self.model = m 
        try:
            self.model.solve(disp=False, debug=True)
            print('Results')
            print('X: ' + str(X.value))
            print('Y: ' + str(Y.value))
            print('T: ' + str(T.value))
            print('Objective: ' + str(m.options.objfcnval))
            self.model.cleanup()
            return [X.value[0], Y.value[0], T.value[0] % 360]
        except Exception as e:
            print(e)
            self.model.cleanup()
            return None

class SingleIntersectionOptimization():

    def __init__(self, intersection, height=1000, width=1800):
        self.intersection = intersection
        self.height = height
        self.width = width
        self.epislon_error = 25
        self.model = None

    
    def solve(self, keyframe_point):
        m = GEKKO() # Initialize gekko
        m.options.SOLVER=1  # APOPT is an MINLP solver. See https://apopt.com/download.php

        # optional solver settings with APOPT
        m.solver_options = ['minlp_maximum_iterations 50000', \
                    # minlp iterations with integer solution
                    'minlp_max_iter_with_int_sol 1000', \
                    # treat minlp as nlp
                    'minlp_as_nlp 1', \
                    # nlp sub-problem max iterations
                    'nlp_maximum_iterations 5000', \
                    # 1 = depth first, 2 = breadth first
                    'minlp_branch_method 2', \
                    # maximum deviation from whole number
                    'minlp_integer_tol 0.1', \
                    # covergence tolerance
                    'minlp_gap_tol 0.01']

        # Initialize variables
        X = m.Var(value=keyframe_point[0])
        Y = m.Var(value=keyframe_point[1])

        # Distance equations

        distance_from_intersection = ((X - self.intersection[0])**2 +
             (Y - self.intersection[1])**2)**0.5

        distance_from_keyframe_point = (
            (X - keyframe_point[0])**2 + (Y - keyframe_point[1])**2)**0.5
        
        m.Equation(X <= self.width)
        m.Equation(X >= 0)
        m.Equation(Y <= self.height)
        m.Equation(Y >= 0)
        m.Equation(distance_from_intersection <= self.epislon_error)
        
        m.Obj(3 * distance_from_intersection + distance_from_keyframe_point)
        self.model = m 
        try:
            self.model.solve(disp=False, debug=True)
            
            print('Results')
            print('X: ' + str(X.value))
            print('Y: ' + str(Y.value))
            print('Objective: ' + str(m.options.objfcnval))
            self.model.cleanup()
            return (X.value[0], Y.value[0])
        except Exception as e:
            print(e)
            self.model.cleanup()
            return None




