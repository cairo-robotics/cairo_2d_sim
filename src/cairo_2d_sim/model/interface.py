import rospy
import pygame as pg

from cairo_2d_sim.display.utils import IMAGE_FILE_DIR
from cairo_2d_sim.msg import ConstraintToggles

class ConstraintOneToggle:
    def __init__(self):
        self.on_image = pg.image.load(IMAGE_FILE_DIR + 'Constraint1On.svg').convert()
        self.on_image = pg.transform.scale(self.on_image, (10, 10))
        self.on_rect = self.on_image.get_rect()

        self.off_image = pg.image.load(IMAGE_FILE_DIR + 'Constraint1Off.svg').convert()
        self.off_image = pg.transform.scale(self.off_image, (10, 10))
        self.off_rect = self.off_image.get_rect()

        self.constraint_sub = rospy.Subscriber('/cairo_2d_sim/constraint_toggles', ConstraintToggles, callback=self.constraint_cb)
        self.on = False

    def render(self, screen):
        if self.on:
            screen.blit(self.on_image, self.on_rect)
            pg.draw.rect(screen, (0,0,0), self.on_rect)
            pg.display.update(self.on_rect)
        else:
            screen.blit(self.off_image, self.off_rect)
            pg.draw.rect(screen, (0,0,0), self.off_rect)
            pg.display.update(self.off_rect)
    
    
    def toggle(self):
        self.on = not self.on
        
    def constraint_cb(self, msg):
        self.on = msg.c1.data


class ConstraintTwoToggle:
    def __init__(self):
        self.on_image = pg.image.load(IMAGE_FILE_DIR + 'Constraint2On.svg').convert()
        self.on_image = pg.transform.scale(self.on_image, (10, 10))
        self.on_rect = self.on_image.get_rect()
        
        self.off_image = pg.image.load(IMAGE_FILE_DIR + 'Constraint2Off.svg').convert()
        self.off_image = pg.transform.scale(self.off_image, (10, 10))
        self.off_rect = self.off_image.get_rect()
    
        self.constraint_sub = rospy.Subscriber('/cairo_2d_sim/constraint_toggles', ConstraintToggles, callback=self.constraint_cb)
        self.on = False

    def render(self, screen):
        if self.on:
            screen.blit(self.on_image, self.on_rect)
            pg.draw.rect(screen, (0,0,0), self.on_rect)
            pg.display.update(self.on_rect)
        else:
            screen.blit(self.off_image, self.off_rect)
            pg.draw.rect(screen, (0,0,0), self.off_rect)
            pg.display.update(self.off_rect)
    
    def toggle(self):
        self.on = not self.on
        
    def constraint_cb(self, msg):
        self.on = msg.c2.data            
            

class ConstraintThreeToggle:
    def __init__(self):
        self.on_image = pg.image.load(IMAGE_FILE_DIR + 'Constraint3On.svg').convert()
        self.on_image = pg.transform.scale(self.on_image, (10, 10))
        self.on_rect = self.on_image.get_rect()
        
        self.off_image = pg.image.load(IMAGE_FILE_DIR + 'Constraint3Off.svg').convert()
        self.off_image = pg.transform.scale(self.off_image, (10, 10))
        self.off_rect = self.off_image.get_rect()
    
        self.constraint_sub = rospy.Subscriber('/cairo_2d_sim/constraint_toggles', ConstraintToggles, callback=self.constraint_cb)
        self.on = False

    def render(self, screen):
        if self.on:
            screen.blit(self.on_image, self.on_rect)
            pg.draw.rect(screen, (0,0,0), self.on_rect)
            pg.display.update(self.on_rect)
        else:
            screen.blit(self.off_image, self.off_rect)
            pg.draw.rect(screen, (0,0,0), self.off_rect)
            pg.display.update(self.off_rect)
    
    def toggle(self):
        self.on = not self.on
    
    def constraint_cb(self, msg):
        self.on = msg.c3.data
            
            