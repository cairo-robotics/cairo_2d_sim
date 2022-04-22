import pygame as pg

from cairo_2d_sim.display.utils import draw_rect_alpha

class RectangleStatic:
    def __init__(self, x, y, width, height, color=None):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color if color is not None else [60, 60, 250, 150]

    def render(self, screen):
        draw_rect_alpha(screen, self.color, (self.x, self.y, self.width, self.height))
