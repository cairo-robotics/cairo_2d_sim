#!/usr/bin/env python
import sys
import physics as p
import pygame as pg
import pid
import rospy
import math as m
from geometry_msgs.msg import Pose, Twist
from cairo_2dsim.msg import KeyPress


class Mouse():
    
    def __init__(self):
        self.x = 0
        self.y = 0
        self.mouse_pub = rospy.Publisher('/mouse', Pose, queue_size=10)
    
    def update(self):
        self.x, self.y = pg.mouse.get_pos()
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        self.position_pub.publish(pose)
        
class XYKeyboard():
    
    def __init__(self):
        self.up = False
        self.down = False
        self.left = False
        self.right = False
        self.mouse_pub = rospy.Publisher('/mouse', Pose, queue_size=10)
    
    def update(self):
        if pg.key.key_code("up") == pg.K_UP:
            self.up = True
        if pg.key.key_code("down") == pg.K_DOWN:
            self.down = True
