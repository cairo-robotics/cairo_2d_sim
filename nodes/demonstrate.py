#! /usr/bin/env python3
import rospy
import sys
import pygame as pg

from cairo_2d_sim.display.environments import BasicEnvironment
from cairo_2d_sim.control.controllers import HolonomicController, InterfaceController
from cairo_2d_sim.model.game import Game
from cairo_2d_sim.model.sprites import HolonomicRobot
from cairo_2d_sim.model.statics import RectangleStatic, CircleStatic
from cairo_2d_sim.model.interface import ConstraintOneToggle, ConstraintTwoToggle, ConstraintThreeToggle


def setup():
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
    
    return statics, toggles, sprites, controllers, env

if __name__ == '__main__':
    pg.init()
    rospy.init_node('game_node', anonymous=True)
    
    HEIGHT = 1800
    WIDTH = 1000
    screen = pg.display.set_mode((HEIGHT, WIDTH))
    pg.display.set_caption("simulation screen")
    
    statics, toggles, sprites, controllers, env = setup()
    # Main game engine and loop
    game_engine = Game(screen, env, sprites, statics, controllers, toggles)
    end_game = False
    while not end_game:
        clock = pg.time.Clock()
        try:
            game_engine.step()
            if game_engine.check_quit():
                end_game = True
            if game_engine.check_restart():
                statics, toggles, sprites, controllers, env = setup()
                # Create new game engine
                game_engine = Game(screen, env, sprites, statics, controllers, toggles)
            if game_engine.check_capture():
                statics, toggles, sprites, controllers, env = setup()
                # Create new game engine becase we need to reset the environment for the next demo
                game_engine = Game(screen, env, sprites, statics, controllers, toggles)
            clock.tick(60)
        except KeyboardInterrupt:
            rospy.signal_shutdown("Demonstration game over.")
            pg.quit()
            sys.exit()