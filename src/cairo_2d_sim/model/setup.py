import pygame as pg

from cairo_2d_sim.display.environments import BasicEnvironment
from cairo_2d_sim.control.controllers import HolonomicController, InterfaceController
from cairo_2d_sim.model.sprites import HolonomicRobot
from cairo_2d_sim.model.statics import RectangleStatic, CircleStatic
from cairo_2d_sim.model.interface import ConstraintOneToggle, ConstraintTwoToggle, ConstraintThreeToggle

def setup():
    HEIGHT = 1800
    WIDTH = 1000
    screen = pg.display.set_mode((HEIGHT, WIDTH))
    # Start and end statically rendered
    start = CircleStatic(100, 500, 15, [0, 255, 0, 150])
    end = CircleStatic(1700, 500, 15, [0, 0, 255, 150])
    
    # Constraint Visuals as statically rendered
    constraint_1 = RectangleStatic(400, 100, 10, 800, (255, 0, 0, 100))
    constraint_2a = RectangleStatic(400, 100, 1000, 10, (0, 255, 0, 100))
    constraint_2b = RectangleStatic(400, 700, 1000, 10, (0, 255, 0, 100))
    constraint_3 = RectangleStatic(1200, 100, 10, 400, (0, 0, 255, 100))
    statics = [start, end, constraint_1, constraint_2a, constraint_2b, constraint_3]
    
    # Constraint toggles
    ct1 = ConstraintOneToggle(1500, 100)
    ct2 = ConstraintTwoToggle(1600, 100)
    ct3 = ConstraintThreeToggle(1700, 100)
    toggles = [ct1, ct2, ct3]
    
    # The only sprite will be the Holonomic Robot
    sprite_1 = HolonomicRobot(100, 500, 90)
    sprites = [sprite_1]
    
    # A single holonomic controller and basic interface controller for toggling constraints
    robot_controller = HolonomicController()
    interface_controller = InterfaceController()
    controllers = [robot_controller, interface_controller]
    
    # Using the BasicEnvironment class for rendering the sandbox environment
    env = BasicEnvironment(WIDTH, HEIGHT)
    
    return screen, statics, toggles, sprites, controllers, env