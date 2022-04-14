#!/usr/bin/env python3
import sys
import pygame as pg


class Display():
    def __init__(self, environment, sprites, statics):
        self.env = environment
        self.sprites = sprites
        self.statics = statics

    def render(self, screen):
        while True:
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    sys.exit()
            self.environment.render(screen)
            for static in self.statics:
                static.render()
            for sprite in self.sprites:
                sprite.render(self.screen)
            pg.display.flip()










