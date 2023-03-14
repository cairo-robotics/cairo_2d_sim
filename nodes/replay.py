#! /usr/bin/env python3
import rospy
import sys
import pygame as pg


from cairo_2d_sim.model.game import Replay
from cairo_2d_sim.model.setup import setup


if __name__ == '__main__':
    pg.init()
    rospy.init_node('game_node', anonymous=True)
    
    pg.display.set_caption("simulation screen")
    
    screen, statics, toggles, sprites, controllers, env = setup()
    # Main game engine and loop
    game_engine = Replay(screen, env, sprites, statics, controllers, toggles)
    end_game = False
    while not end_game:
        clock = pg.time.Clock()
        try:
            game_engine.step()
            if game_engine.check_quit():
                end_game = True
            if game_engine.check_restart():
                screen, statics, toggles, sprites, controllers, env = setup()
                # Create new game engine
                game_engine = Replay(screen, env, sprites, statics, controllers, toggles)
            if game_engine.check_capture():
                screen, statics, toggles, sprites, controllers, env = setup()
                # Create new game engine becase we need to reset the environment for the next demo
                game_engine = Replay(screen, env, sprites, statics, controllers, toggles)
            clock.tick(60)
        except KeyboardInterrupt:
            rospy.signal_shutdown("Replay over.")
            pg.quit()
            sys.exit()
    rospy.signal_shutdown("Replay over.")
    pg.quit()
    sys.exit()