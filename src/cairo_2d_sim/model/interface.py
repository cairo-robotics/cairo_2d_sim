import rospy
import pygame as pg

from cairo_2d_sim.display.utils import IMAGE_FILE_DIR
from cairo_2d_sim.msg import ConstraintToggles

class ConstraintOneToggle:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
        self.on_image = pg.image.load(IMAGE_FILE_DIR + 'Constraint1On.jpg').convert_alpha()
        self.on_image = pg.transform.scale(self.on_image, (75, 75))


        self.off_image = pg.image.load(IMAGE_FILE_DIR + 'Constraint1Off.jpg').convert_alpha()
        self.off_image = pg.transform.scale(self.off_image, (75, 75))

        self.constraint_sub = rospy.Subscriber('/cairo_2d_sim/constraint_toggles', ConstraintToggles, callback=self.constraint_cb)
        self.on = False

    def render(self, screen):
        if self.on:
            screen.blit(self.on_image, (self.x, self.y))
        else:
            screen.blit(self.off_image,(self.x, self.y))
    
    def toggle(self):
        self.on = not self.on
        
    def constraint_cb(self, msg):
        if msg.c1.data:
            self.toggle()


class ConstraintTwoToggle:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
        self.on_image = pg.image.load(IMAGE_FILE_DIR + 'Constraint2On.jpg').convert_alpha()
        self.on_image = pg.transform.scale(self.on_image, (75, 75))
        
        self.off_image = pg.image.load(IMAGE_FILE_DIR + 'Constraint2Off.jpg').convert_alpha()
        self.off_image = pg.transform.scale(self.off_image, (75, 75))
    
        self.constraint_sub = rospy.Subscriber('/cairo_2d_sim/constraint_toggles', ConstraintToggles, callback=self.constraint_cb)
        self.on = False

    def render(self, screen):
        if self.on:
            screen.blit(self.on_image, (self.x, self.y))
        else:
            screen.blit(self.off_image,(self.x, self.y))
    
    def toggle(self):
        self.on = not self.on
        
    def constraint_cb(self, msg):
        if msg.c2.data:
            self.toggle()       
            

class ConstraintThreeToggle:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
        self.on_image = pg.image.load(IMAGE_FILE_DIR + 'Constraint3On.jpg').convert()
        self.on_image = pg.transform.scale(self.on_image, (75, 75))
 
        self.off_image = pg.image.load(IMAGE_FILE_DIR + 'Constraint3Off.jpg').convert()
        self.off_image = pg.transform.scale(self.off_image, (75, 75))
    
        self.constraint_sub = rospy.Subscriber('/cairo_2d_sim/constraint_toggles', ConstraintToggles, callback=self.constraint_cb)
        self.on = False

    def render(self, screen):
        if self.on:
            screen.blit(self.on_image, (self.x, self.y))
        else:
            screen.blit(self.off_image,(self.x, self.y))
    
    def toggle(self):
        self.on = not self.on
    
    def constraint_cb(self, msg):
        if msg.c3.data:
            self.toggle()
            
            