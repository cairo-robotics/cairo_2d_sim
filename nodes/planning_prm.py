#! /usr/bin/env python3
import numpy as np
import os
from functools import partial
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import  Pose2D
import pygame as pg

from cairo_2d_sim.msg import Pose2DStamped
from cairo_2d_sim.planning.state_space import Holonomic2DStateSpace, StateValidityChecker
from cairo_2d_sim.planning.constraints import UnconstrainedPRMTSR, LineConstraintPRMTSR, LineTargetingConstraintPRMTSR
from cairo_2d_sim.planning.planners import CPRM
from cairo_2d_sim.planning.curve import JointTrajectoryCurve, xytheta_distance, parametric_xytheta_lerp


FILE_DIR = os.path.dirname(os.path.realpath(__file__))


if __name__ == '__main__':
    rospy.init_node("Planning publisher")
    
    start_q = [405, 100, 277.72]
    goal_q = [800, 100, 225.20]
    
    state_space = Holonomic2DStateSpace((0, 1800), (0, 1000))
    svc = StateValidityChecker(start_q, goal_q)
    interp_fn = partial(parametric_xytheta_lerp, steps=10)
    prm = CPRM(state_space, svc, interp_fn, xytheta_distance, {'smooth_path': False, 'ball_radius': 50, 'n_samples': 12000, 'k': 15})
    
    tsr = UnconstrainedPRMTSR()
    path_points = prm.get_path(prm.plan(tsr, start_q, goal_q))
    
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
