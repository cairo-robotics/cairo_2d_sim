
import pygame as pg

from cairo_2d.display.display import Display

class Game():
    
    def __init__(self, screen, environment, sprites, statics):
        self.display = Display(screen, environment, sprites, statics)
        self.sprites = sprites
        self.statics
        
    def update(self):
        for sprite in self.sprites:
            sprite.update()
            
    def render(self):
        self.display.render()
            
    def step(self):
        self.update()
        self.render()
 
    def run(self):
        clock = pg.time.Clock()
        self.tick()
        clock.tick(60)
    
 
        
    
    