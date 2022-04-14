import rospy

import pygame as pg

from display.utils import text_objects

class HolonomicRobot(pg.sprite.Sprite):
    """
    A sprite that can move in any direction.
    """
    def __init__(self, image, x, y):
        super().__init__()
        self.image = image
        self.name = 'robot_'+ name + '/pose'
        self.control_subscriber = rospy.Subscriber(self.name, Pose, self._cmd_callback)
        self.radius = 40
        self.x_pos = int(xp)
        self.y_pos = int(yp)
        self.yaw = yaw
        self.color = [255, 0, 0]

    def render(self, screen):
        pg.draw.circle(self.screen, self.color, [self.x, self.y], self.radius, 0)
        pg.draw.circle(self.screen, [0, 0, 0], [
                       self.x+int(20*m.cos(self.yaw)), self.y+int(20*m.sin(self.yaw))], 10, 0)
        font = pg.font.Font('freesansbold.ttf', 20)
        txtsf, txtre = text_objects(str(self.n), font)
        txtre.center = (self.x, self.y)
        screen.blit(txtsf, txtre)
    
    def move(self, dx, dy, dx_yaw):
        self.rect.x += dx
        self.rect.y += dy
    
    
    def update(self):
        pass
    
    def _cmd_callback(self, msg):
        self.x_pos = msg.position.x
        self.y_pos = msg.position.y
        quat = [msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quat)
        self.yaw = euler[2]
        [self.xo, self.yo, self.yaw] = disptf(self.xo, self.yo, self.yaw)
    
