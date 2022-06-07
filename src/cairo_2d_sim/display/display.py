#!/usr/bin/env python3
import pygame as pg


class Display():
    def __init__(self, environment, sprites, statics, toggles):
        self.env = environment
        self.sprites = sprites
        self.statics = statics
        self.toggles = toggles

    def render(self, screen):
        self.env.render(screen)
        for static in self.statics:
            static.render(screen)
        for sprite in self.sprites:
            sprite.render(screen)
        for toggle in self.toggles:
            toggle.render(screen)
        pg.display.flip()
    
    def update_statics(self, statics):
        self.statics = statics










