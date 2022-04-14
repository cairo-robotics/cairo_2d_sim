import rospy
import pygame as pg

from cairo_2d.display.environment import BasicEnvironment
from cairo_2d.model.game import Game
from cairo_2d.model.sprites import HolonomicRobot
from cairo_2d.model.statics import Static 

if __name__ == '__main__':
    pg.init()
    screen = pg.display.set_mode((1000, 760))
    pg.display.set_caption("simulation screen")
    sprites = [HolonomicRobot(0, 0, 10, 10, (255, 0, 0))]
    Static(0, 0, 10, 10, (0, 255, 0))
    statics = []
    env = BasicEnvironment(screen)
    display = Game(screen, env, sprites)
    rospy.init_node('game_node', anonymous=True)
    try:
        display.render()
    except KeyboardInterrupt:
        exit()

def game(t1,t2):
    global r10msg
    global r11msg
    global r20msg
    global r21msg
    global d
    global gs
    global r1f
    global r2f
    global r3f
    global r4f
    rospy.Subscriber('game/status',Int32,rulecheck)
    robotsubinit()
    pubball = rospy.Publisher('ballpose', Pose, queue_size=10)
    pubbtwist = rospy.Publisher('balltwist', Twist, queue_size=10)
    drib = rospy.Publisher('game/dribbler', Int32, queue_size=10)
    yis = rospy.Publisher('game/dribdist', Float64, queue_size=10)
    pr1 = []
    pr2 = []
    a,r1r = robotpubinit(1,0)
    pr1.append(a)
    a,r2r = robotpubinit(1,1)
    pr1.append(a)
    a,r3r = robotpubinit(2,0)
    pr2.append(a)
    a,r4r = robotpubinit(2,1)
    pr2.append(a)
    btwist = Twist()
    rate = rospy.Rate(60)
    while True:
        ball = p.ball(x = 0,y = 0)
        bpose = Pose()
        r1 = []
        r2 = []
        r1.append(p.robot(x= t1[0][0],y= t1[0][1], yaw  = 0, ball = ball))
        r1.append(p.robot(x= t1[1][0],y= t1[1][1], yaw  = 0, ball = ball))
        r2.append(p.robot(x= t2[0][0],y= t2[0][1], yaw  = 3.14, ball = ball))
        r2.append(p.robot(x= t2[1][0],y= t2[1][1], yaw  = 3.14, ball = ball))

        rpose = [Pose(),Pose(),Pose(),Pose()]
        updatebpose(bpose,ball)
        updatebtwist(btwist,ball)
        updaterpose(rpose[0],r1[0])
        updaterpose(rpose[1],r1[1])
        updaterpose(rpose[2],r2[0])
        updaterpose(rpose[3],r2[1])
        pr1[0].publish(rpose[0])
        pr1[1].publish(rpose[1])
        pr2[0].publish(rpose[2])
        pr2[1].publish(rpose[3])
        pubball.publish(bpose)
        while not rospy.is_shutdown():
            if gs == 0:
                c.control(r10msg,r1[0],ball)
                c.control(r11msg,r1[1],ball)
                c.control(r20msg,r2[0],ball)
                c.control(r21msg,r2[1],ball)
                p.collRR(r1[0],r2[0])
                p.collRR(r1[0],r2[1])
                p.collRR(r1[0],r1[1])
                p.collRR(r1[1],r2[0])
                p.collRR(r1[1],r2[1])
                p.collRR(r2[0],r2[1])
                dribbletest(r1[0 ],r1[1],r2[0],r2[1])
                updatebpose(bpose,ball)
                updatebtwist(btwist,ball)
                x1 = updaterpose(rpose[0],r1[0])
                x2 = updaterpose(rpose[1],r1[1])
                x3 = updaterpose(rpose[2],r2[0])
                x4 = updaterpose(rpose[3],r2[1])
                x = [x1,x2,x3,x4]
                y = max(x)
                yis.publish(y)
                r1r.publish(r1f)
                r2r.publish(r2f)
                r3r.publish(r3f)
                r4r.publish(r4f)
                pr1[0].publish(rpose[0])
                pr1[1].publish(rpose[1])
                pr2[0].publish(rpose[2])
                pr2[1].publish(rpose[3])
                pubball.publish(bpose)
                pubbtwist.publish(btwist)
                drib.publish(d)
                rate.sleep()
            else:
                dribbletest(r1[0],r1[1],r2[0],r2[1])
                updatebpose(bpose,ball)
                updatebtwist(btwist,ball)
                x1 = updaterpose(rpose[0],r1[0])
                x2 = updaterpose(rpose[1],r1[1])
                x3 = updaterpose(rpose[2],r2[0])
                x4 = updaterpose(rpose[3],r2[1])
                x = [x1,x2,x3,x4]
                y = max(x)
                yis.publish(y)
                r1r.publish(r1f)
                r2r.publish(r2f)
                r3r.publish(r3f)
                r4r.publish(r4f)
                pr1[0].publish(rpose[0])
                pr1[1].publish(rpose[1])
                pr2[0].publish(rpose[2])
                pr2[1].publish(rpose[3])
                pubball.publish(bpose)
                pubbtwist.publish(btwist)
                drib.publish(d)
                rate.sleep()
                break