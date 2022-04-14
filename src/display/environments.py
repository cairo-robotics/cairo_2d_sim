import pygame as pg

class BasicEvironment():
        
    def __init__(self, width, height, floor_color):
        self.width = width
        self.height = height
        self.floor_color = floor_color if floor_color is not None else [60, 60, 60]
        
    def render(self, screen):
        screen.fill((0, 0, 0))
        # field
        pg.draw.rect(screen, self.floor_color, (20, 20, self.width, self.height))
        # bounds
        pg.draw.rect(screen, [255, 255, 255], (20, 20, self.width, self.height), 8)
                
    
    @property
    def description(self):
        return "Basic enviroment for testing."
    
    @property
    def dimensions(self):
        return self.width, self.height