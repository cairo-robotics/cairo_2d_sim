from math import pi, atan2

import rospy
from geometry_msgs.msg import Pose, Pose2D
import pygame as pg

from cairo_2d_sim.display.utils import IMAGE_FILE_DIR
from cairo_2d_sim.state.utils import offset
from cairo_2d_sim.msg import KeyboardArrows, MousePress


class HolonomicRobot(pg.sprite.Sprite):
    """
    A sprite that can move in any direction.
    """

    def __init__(self, x_init, y_init, yaw, image_fp=None):
        super().__init__()
        self.image = pg.image.load(image_fp).convert() if image_fp is not None else pg.image.load(IMAGE_FILE_DIR + 'millennium-falcon.svg')
        self.image = pg.transform.scale(self.image, (75, 75))
        self.image = pg.transform.rotate(self.image, -90)
        self.image.get_rect(center=(x_init, y_init))
        self.x_pos = int(x_init)
        self.y_pos = int(y_init)
        self.old_x_pos = int(x_init)
        self.old_y_pos = int(y_init)
        self.yaw = yaw
        self.new_yaw = yaw
        self.left_click = False
        self.color = [255, 255, 255]
        self.trace_color = [0, 0, 0]  
        self.dx = 0
        self.dy = 0
        self.pos_dx = 2
        self.points = []
        self.points_count = 0
        self.keyboard_arrow_sub = rospy.Subscriber('/cairo_2d_sim/direction_commands', KeyboardArrows, self._keyboard_cb)
        self.mouse_pos_sub = rospy.Subscriber('/cairo_2d_sim/mouse_posistion', Pose, self._mouse_pos_cb)
        self.mouse_press_sub = rospy.Subscriber('/cairo_2d_sim/mouse_press', MousePress, self._mouse_press_cb)
        self.state_pub = rospy.Publisher('/cairo_2d_sim/robot_state', Pose2D, queue_size=1)
    
    def update(self):
        self._update_xy()
        self._update_yaw()
        self._update_points()
        pose2d = Pose2D()
        pose2d.x = self.x_pos
        pose2d.y = self.y_pos
        pose2d.theta = self.yaw
        self.state_pub.publish(pose2d)
    
    def render(self, screen):
        rotimage = pg.transform.rotate(self.image, self.yaw)
        rect = rotimage.get_rect(center=(self.x_pos, self.y_pos))
        for p1, p2 in offset(self.points):
            if p1 and p2:
                pg.draw.line(screen, (0,0,0), p1, p2)
        screen.blit(rotimage, rect) 

    def _update_xy(self):
        self.x_pos += self.dx
        self.y_pos += self.dy
    
    def _update_yaw(self):
        if self.left_click:
            self.yaw = self.new_yaw
    
    def _update_points(self):
        if self.points_count % 10 == 0:
            self.points.append((self.x_pos, self.y_pos))
        
    def _keyboard_cb(self, msg):
        if msg.left.data:
            self.dx = -self.pos_dx
        if msg.right.data:
            self.dx = self.pos_dx
        if msg.up.data:
            self.dy = -self.pos_dx
        if msg.down.data:
            self.dy = self.pos_dx
        if msg.left.data and msg.right.data:
            self.dx = 0
        if msg.up.data and msg.down.data:
            self.dy = 0
        if not msg.left.data and not msg.right.data:
            self.dx = 0
        if not msg.up.data and not msg.down.data:
            self.dy = 0
        
    def _mouse_pos_cb(self, msg):
        mouse_x = msg.position.x
        mouse_y = msg.position.y
        angle = 360 - atan2(mouse_y - self.y_pos, mouse_x - self.x_pos) * 180 / pi
        self.new_yaw = angle
    
    def _mouse_press_cb(self, msg):
        self.left_click = msg.left.data
