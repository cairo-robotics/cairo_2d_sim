import pygame as pg


class RectangleStatic:
    def __init__(self, x, y, width, height, color=None):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color if color is not None else [60, 60, 250]

    def render(self, screen):
        pg.draw.rect(screen, self.color, (self.x, self.y, self.width, self.height))