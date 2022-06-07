import pygame as pg

from cairo_2d_sim.display.utils import draw_rect_alpha, draw_circle_alpha, draw_arrow_alpha

class RectangleStatic:
    def __init__(self, x, y, width, height, color=None):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color if color is not None else [60, 60, 250, 150]

    def render(self, screen):
        draw_rect_alpha(screen, self.color, (self.x, self.y, self.width, self.height))
        

class DirectionalCircleStatic:
    def __init__(self, x, y, angle, radius, color=None):
        self.x = x
        self.y = y
        self.angle = angle
        self.radius = radius
        self.color = color if color is not None else [60, 60, 250, 150]
    
    def render(self, screen):
        draw_circle_alpha(screen, self.color, (self.x, self.y), self.radius)
        draw_arrow_alpha(screen, [0, 0, 0], (self.x, self.y), self.angle)
        

class CircleStatic:
    def __init__(self, x, y, radius, color=None):
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color if color is not None else [60, 60, 250, 150]

    def render(self, screen):
        draw_circle_alpha(screen, self.color, (self.x, self.y), self.radius)
