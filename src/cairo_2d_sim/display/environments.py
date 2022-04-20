import pygame as pg

class BasicEnvironment():
        
    def __init__(self, height, width, floor_color=None):
        self.width = width
        self.height = height
        self.floor_color = floor_color if floor_color is not None else [255, 255, 255]
        
    def render(self, screen):
        screen.fill((0, 0, 0))
        # field
        pg.draw.rect(screen, self.floor_color, (0, 0, self.width, self.height))
        # bounds
        pg.draw.rect(screen, [0, 0, 0], (0, 0, self.width, self.height), 8)
                
    
    @property
    def description(self):
        return "Basic enviroment for testing."
    
    @property
    def dimensions(self):
        return self.width, self.height