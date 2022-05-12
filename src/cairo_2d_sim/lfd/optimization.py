from itertools import tee
from pyomo.environ import *


class Constraint_1_2():
    
    def __init__(self, first_intersection, second_intersection, height=1000, width=1800):
        self.first_intersection = first_intersection
        self.second_intersection = second_intersection
        self.height = height
        self.width = width
        self.model = None
    
    def generate_model(self, keyframe_point):
        model = ConcreteModel()

        model.X = Var()
        model.Y = Var()
        model.A = Var(within=Binary)

        model.C1 = Constraint(expr = model.X <= self.width)
        model.C2 = Constraint(expr = model.X >= 0)
        model.C3 = Constraint(expr = model.Y <= self.height)
        model.C4 = Constraint(expr = model.Y >= 0)

        distance_from_first_intersection = model.A * ((model.X - self.first_intersection[0])**2 + (model.Y - self.first_intersection[1])**2)**0.5
        
        distance_from_second_intersection = (1-model.A) * ((model.X - self.second_intersection[0])**2 + (model.Y - self.second_intersection[1])**2)**0.5
        
        distance_from_intersection_combined = distance_from_first_intersection + distance_from_second_intersection
        
        distance_from_keyframe_point = ((model.X - keyframe_point[0])**2 + (model.Y - keyframe_point[1])**2)**0.5
        
        model.obj_1 = Objective(expr = distance_from_intersection_combined + distance_from_keyframe_point, sense=minimize)

        self.model = model
    
    def solve(self):
        if self.model is not None:
            results = SolverFactory('mindtpy').solve(self.model, mip_solver='glpk', nlp_solver='ipopt', tee=True)   #'cplex', 'ipopt'

            return results, self.model.X(), self.model.Y(), self.model.A()
        else:
            raise Exception("Call generate_mode() before solve()")