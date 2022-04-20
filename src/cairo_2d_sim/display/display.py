#!/usr/bin/env python3
import sys
import pygame as pg


class Display():
    def __init__(self, environment, sprites, statics):
        self.env = environment
        self.sprites = sprites
        self.statics = statics

    def render(self, screen):
        self.env.render(screen)
        for static in self.statics:
            static.render(screen)
        for sprite in self.sprites:
            sprite.render(screen)
        pg.display.flip()










