from itertools import tee
from pyomo.environ import *
from gekko import GEKKO


class Constraint_1_2():

    def __init__(self, first_intersection, second_intersection, height=1000, width=1800):
        self.first_intersection = first_intersection
        self.second_intersection = second_intersection
        self.height = height
        self.width = width
        self.epislon_error = 100
        self.model = None

    def generate_model(self, keyframe_point):
        model = ConcreteModel()

        model.X = Var(within=NonNegativeReals)
        model.Y = Var(within=NonNegativeReals)
        model.A = Var(domain=Binary)
        
        distance_from_first_intersection = model.A * ((model.X - self.first_intersection[0])**2 +
             (model.Y - self.first_intersection[1])**2)**0.5

        distance_from_second_intersection = (
            1-model.A) * ((model.X - self.second_intersection[0])**2 + (model.Y - self.second_intersection[1])**2)**0.5

        distance_from_intersection_combined = distance_from_first_intersection + \
            distance_from_second_intersection

        distance_from_keyframe_point = (
            (model.X - keyframe_point[0])**2 + (model.Y - keyframe_point[1])**2)**0.5

        model.C1 = Constraint(expr = model.X <= self.width)
        # model.C2 = Constraint(expr = model.X >= 0)
        model.C3 = Constraint(expr = model.Y <= self.height)
        # model.C4 = Constraint(expr = model.Y >= 0)
        model.C5 = Constraint(expr = model.A * distance_from_first_intersection <= self.epislon_error)
        model.C6 = Constraint(expr = (1 - model.A) * distance_from_second_intersection <= self.epislon_error)




        model.obj_1 = Objective(
            expr = distance_from_intersection_combined + distance_from_keyframe_point, sense=minimize)

        self.model = model

    def solve(self):
        if self.model is not None:
            results = SolverFactory('mindtpy').solve(self.model,
                                                     mip_solver='glpk',
                                                     nlp_solver='ipopt',
                                                     tee=True)  # 'cplex', 'ipopt'

            return results, self.model.X(), self.model.Y(), self.model.A()
        else:
            raise Exception("Call generate_model() before solve()")

class Constraint_1_2_gekko():

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
            print('x1: ' + str(X.value))
            print('x2: ' + str(Y.value))
            print('x3: ' + str(A.value))
            print('Objective: ' + str(m.options.objfcnval))
            self.model.cleanup()
        except Exception as e:
            print(e)
            self.model.cleanup()

