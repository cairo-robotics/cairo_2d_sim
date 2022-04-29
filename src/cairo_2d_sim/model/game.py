
import sys, signal

import pygame as pg

from cairo_2d_sim.display.display import Display



def signal_handler(signal, frame):
  pg.quit()
  sys.exit(0)




class Game():
    
    def __init__(self, screen, environment, sprites, statics, controllers, toggles):
        self.screen = screen
        self.display = Display(environment, sprites, statics, toggles)
        self.sprites = sprites
        self.statics = statics
        self.controllers = controllers
        self.toggles = toggles
        
    def _update_toggles(self):
        for toggle in self.toggles:
            toggle.update()
        
    def _update_controls(self, event):
        for controller in self.controllers:
            controller.update(event)
    
    def _update_sprites(self):
        for sprite in self.sprites:
            sprite.update()
            
    def _render(self):
        self.display.render(self.screen)
            
    def run(self):
        clock = pg.time.Clock()
        continue_loop = True
        while continue_loop:

            for event in pg.event.get():
                if event.type == pg.QUIT:
                    continue_loop = False
                self._update_controls(event)
            self._update_toggles()
            self._update_sprites()
            self._render()
            clock.tick(60)
            signal.signal(signal.SIGTERM, signal_handler)
            signal.signal(signal.SIGINT, signal_handler)


        
    
 
        
    
    