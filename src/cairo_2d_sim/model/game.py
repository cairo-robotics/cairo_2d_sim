
import pygame as pg

from cairo_2d_sim.display.display import Display

class Game():
    
    def __init__(self, screen, environment, sprites, statics, controllers, toggles):
        self.screen = screen
        self.display = Display(environment, sprites, statics, toggles)
        self.sprites = sprites
        self.statics = statics
        self.controllers = controllers
        self.toggles = toggles
        
    def update(self):
        for controller in self.controllers:
            controller.update()
        for sprite in self.sprites:
            sprite.update()
            
    def render(self):
        self.display.render(self.screen)
            
    def step(self):
        self.update()
        self.render()
 
    def run(self):
        clock = pg.time.Clock()
        self.step()
        clock.tick(60)
    
 
        
    
    