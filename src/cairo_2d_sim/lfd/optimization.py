from pyomo.environ import *
import matplotlib.pyplot as plt


class Constraint_1_2():
    
    def __init__(self, intersection_x, intersection_y, height=1800, width=1000):
        self.intersection = (intersection_x, intersection_y)
        self.height = height
        self.width = width
    
    def generate_model(self, keyframe_point):
        model = ConcreteModel()

        model.x = Var(within=NonNegativeReals)
        model.y = Var(within=NonNegativeReals)

        model.a = Var(within=Binary)

        model.C1 = Constraint(expr = model.x <= self.height)
        model.C2 = Constraint(expr = model.x >= 0)
        model.C3 = Constraint(expr = model.y <= self.width)
        model.C4 = Constraint(expr = model.y <= 0)

        distance_from_intersection = ((model.x - self.intersection[0])**2 + (model.y - self.intersection[1])**2)**0.5
        distance_from_keyframe_point = ((model.x - keyframe_point[0])**2 + (model.y - keyframe_point[1])**2)**0.5
        
        model.obj_1 = Objective(expr = distance_from_intersection, sense=minimize)
        model.obj_2 = Objective(expr = distance_from_keyframe_point, sense=minimize)

        return model
    
    def solve(self, model):
        solver = SolverFactory('mindtpy').solve(model, mip_solver='glpk', nlp_solver='ipopt')   #'cplex', 'ipopt'
        solver.solve(model)