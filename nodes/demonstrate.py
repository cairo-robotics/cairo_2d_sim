#! /usr/bin/env python3
import rospy
import json
import pygame as pg

from cairo_2d_sim.display.environments import BasicEnvironment
from cairo_2d_sim.control.controllers import HolonomicController, InterfaceController
from cairo_2d_sim.model.game import Game
from cairo_2d_sim.model.sprites import HolonomicRobot
from cairo_2d_sim.model.statics import RectangleStatic, CircleStatic
from cairo_2d_sim.model.interface import ConstraintOneToggle, ConstraintTwoToggle, ConstraintThreeToggle

if __name__ == '__main__':
    pg.init()
    HEIGHT = 1800
    WIDTH = 1000
    screen = pg.display.set_mode((HEIGHT, WIDTH))
    pg.display.set_caption("simulation screen")
    
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
    ct1 = ConstraintOneToggle()
    ct2 = ConstraintTwoToggle()
    ct3 = ConstraintThreeToggle()
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
    
    # Main game engine and loop
    game_engine = Game(screen, env, sprites, statics, controllers, toggles)
    rospy.init_node('game_node', anonymous=True)
    try:
        while not rospy.is_shutdown():
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    exit()
            game_engine.run()
    except KeyboardInterrupt:
        exit()