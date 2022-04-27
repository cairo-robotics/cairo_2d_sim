import pygame as pg

class BasicEnvironment():
        
    def __init__(self, height, width, fill_color=None):
        self.width = width
        self.height = height
        self.fill_color = fill_color if fill_color is not None else [255, 255, 255]
        
    def render(self, screen):
        screen.fill(self.fill_color)
        # field
        pg.draw.rect(screen, self.fill_color, (0, 0, self.width, self.height))
        # bounds
        pg.draw.rect(screen, [0, 0, 0], (0, 0, self.width, self.height), 8)
                
    
    @property
    def description(self):
        return "Basic enviroment for testing."
    
    @property
    def dimensions(self):
        return self.width, self.height