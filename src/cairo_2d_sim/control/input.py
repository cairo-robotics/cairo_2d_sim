#!/usr/bin/env python
import sys
import pygame as pg
import rospy
import math as m
from geometry_msgs.msg import Pose
from cairo_2d_sim.msg import ConstraintToggles, DirectionCommands, MousePress, MenuCommands


class MousePositionInput():
    
    def __init__(self):
        self.x = 0
        self.y = 0
        self.mouse_position_pub = rospy.Publisher('/cairo_2d_sim/mouse_position', Pose, queue_size=1)
    
    def update(self, _):
        self.x, self.y = pg.mouse.get_pos()
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        self.mouse_position_pub.publish(pose)
    
class MousePressInput():
    
    def __init__(self):
        self.left = 0
        self.middle = 0
        self.right = 0
        self.mouse_press_pub = rospy.Publisher('/cairo_2d_sim/mouse_press', MousePress, queue_size=1)
    
    def update(self, _):
        self.left, self.middle, self.right = pg.mouse.get_pressed()
        mouse_press = MousePress()
        mouse_press.left.data = True if self.left == 1 else False
        mouse_press.middle.data = True if self.middle == 1 else False
        mouse_press.right.data = True if self.right == 1 else False
        self.mouse_press_pub.publish(mouse_press)
        
class KeyboardArrowsInput():
    
    def __init__(self):
        self.up = False
        self.down = False
        self.left = False
        self.right = False
        self.keyboard_pub = rospy.Publisher('/cairo_2d_sim/direction_commands', DirectionCommands, queue_size=1)
    
    def update(self, _):
        self.up = pg.key.get_pressed()[pg.K_UP]
        self.down = pg.key.get_pressed()[pg.K_DOWN]
        self.left = pg.key.get_pressed()[pg.K_LEFT]
        self.right = pg.key.get_pressed()[pg.K_RIGHT]
        key_press = DirectionCommands()
        key_press.up.data = self.up
        key_press.down.data = self.down
        key_press.right.data = self.right
        key_press.left.data = self.left
        self.keyboard_pub.publish(key_press)
        
class ConstraintTogglesInput():
    
    def __init__(self):
        self.c1 = False
        self.c2 = False
        self.c3 = False
        self.keyboard_pub = rospy.Publisher('/cairo_2d_sim/constraint_toggles', ConstraintToggles, queue_size=1)
        
    def update(self, event):
        if event is not None:
            if event.type == pg.KEYDOWN:
                if event.key == pg.K_1:
                    self.c1 = True
                else:
                    self.c1 = False
                if event.key == pg.K_2:
                    self.c2 = True
                else:
                    self.c2 = False
                if event.key == pg.K_3:
                    self.c3 = True
                else:
                    self.c3 = False
                ct = ConstraintToggles()
                ct.c1.data = self.c1
                ct.c2.data = self.c2
                ct.c3.data = self.c3
                self.keyboard_pub.publish(ct)

        
class MenuCommandsInput():
    
    def __init__(self):
        self.keyboard_pub = rospy.Publisher('/cairo_2d_sim/menu_commands', MenuCommands, queue_size=1)
        self.capture = False
        self.restart = False
        self.quit = False
        
    def update(self, event):
        if event is not None:
            if event.type == pg.KEYDOWN:
                if event.key == pg.K_c:
                    self.capture = True
                else:
                    self.capture = False
                if event.key == pg.K_r:
                    self.restart = True
                else:
                    self.restart = False
                if event.key == pg.K_q:
                    self.quit = True
                else:
                    self.quit = False
                mc = MenuCommands()
                mc.capture.data = self.capture
                mc.restart.data = self.restart
                mc.quit.data = self.quit
                self.keyboard_pub.publish(mc)



        
            
        
