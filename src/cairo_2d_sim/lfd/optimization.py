from pyomo.environ import *
import matplotlib.pyplot as plt

# max 			f1 = X1
# max 			f2 = 3 X1 + 4 X2
# constraints	X1 <= 20
#     			X2 <= 40
# 				5 X1 + 4 X2 <= 200

class BoxConstraint():
    
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        
    def build_constraint(self, model):
        model.box_constraint1 = Constraint(expr=self.x <= self.width)
        model.box_constraint = Constraint(expr=self.y <= self.height)

# model = ConcreteModel()

# model.X1 = Var(within=NonNegativeReals)
# model.X2 = Var(within=NonNegativeReals)

# model.C1 = Constraint(expr = model.X1 <= 20)
# model.C2 = Constraint(expr = model.X2 <= 40)
# model.C3 = Constraint(expr = 5 * model.X1 + 4 * model.X2 <= 200)

# model.f1 = Var()
# model.f2 = Var()
# model.C_f1 = Constraint(expr = model.f1 == model.X1)

# C_f2 = Constraint(expr = model.f2 == 3 * model.X1 + 4 * model.X2)
# model.O_f1 = Objective(expr = model.f1, sense=maximize)
# model.O_f2 = Objective(expr = model.f2, sense=maximize)

# solver = SolverFactory('glpk')  #'cplex', 'ipopt'
# solver.solve(model)