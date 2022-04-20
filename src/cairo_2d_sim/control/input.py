#!/usr/bin/env python
import sys
import pygame as pg
import rospy
import math as m
from geometry_msgs.msg import Pose
from cairo_2d_sim.msg import KeyPress, MousePress


class MousePositionInput():
    
    def __init__(self):
        self.x = 0
        self.y = 0
        self.mouse_position_pub = rospy.Publisher('/cairo_2d_sim/mouse_pos', Pose, queue_size=1)
    
    def update(self):
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
    
    def update(self):
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
        self.keyboard_pub = rospy.Publisher('/cairo_2d_sim/keyboard', KeyPress, queue_size=1)
    
    def update(self):
        self.up = pg.key.get_pressed()[pg.K_UP]
        self.down = pg.key.get_pressed()[pg.K_DOWN]
        self.left = pg.key.get_pressed()[pg.K_LEFT]
        self.right = pg.key.get_pressed()[pg.K_RIGHT]
        key_press = KeyPress()
        key_press.up.data = self.up
        key_press.down.data = self.down
        key_press.right.data = self.right
        key_press.left.data = self.left
        self.keyboard_pub.publish(key_press)

        
            
        
