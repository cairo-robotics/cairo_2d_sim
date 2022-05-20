#! /usr/bin/env python3
import numpy as np
import os
from functools import partial
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import  Pose2D
import pygame as pg

from cairo_2d_sim.msg import Pose2DStamped
from cairo_2d_sim.lfd.state_space import Holonomic2DStateSpace, StateValidityChecker
from cairo_2d_sim.planning.constraints import UnconstrainedTreeTSR, LineConstraintTreeTSR, LineTargetingConstraintTreeTSR
from cairo_2d_sim.planning.planners import CRRT
from cairo_2d_sim.planning.curve import JointTrajectoryCurve, xytheta_distance, parametric_xytheta_lerp


FILE_DIR = os.path.dirname(os.path.realpath(__file__))


if __name__ == '__main__':
    rospy.init_node("Planning publisher")
    
    state_space = Holonomic2DStateSpace((0, 1800), (0, 1000))
    svc = StateValidityChecker()
    interp_fn = partial(parametric_xytheta_lerp, steps=10)
    crrt = CRRT(state_space, svc, interp_fn, xytheta_distance, {'smooth_path': False, 'epsilon': 200, 'e_step': .25, 'smoothing_time': 10})
    
    start_q = [405, 100, 277.67]
    goal_q = [405, 800, 225.20]
    
    tsr = LineConstraintTreeTSR([405, 100], [1200, 100])
    path_points = crrt.get_path(crrt.plan(tsr, start_q, goal_q))
    
    print(path_points)
    for point in path_points:
        print(point)

    move_time = 10

    trajectory = list(zip([move_time * n/len(path_points) for n in range(0, len(path_points))], [q[0] for q in path_points], [q[1] for q in path_points], [q[2] for q in path_points]))
    
    print(trajectory)
    
    state_pub = rospy.Publisher('/cairo_2d_sim/robot_state_replay', Pose2DStamped, queue_size=1)
    
    prior_time = trajectory[0][0]
    for point in trajectory:
        header = Header()
        header.stamp = rospy.Time.now()
        pose2d = Pose2D()
        pose2d.x = point[1]
        pose2d.y = point[2]
        pose2d.theta = point[3]
        pose2dstamped = Pose2DStamped()
        pose2dstamped.header = header
        pose2dstamped.pose2d = pose2d
        state_pub.publish(pose2dstamped)
        rospy.sleep(point[0] - prior_time)
        prior_time =point[0]
