#! /usr/bin/env python3
import json
import os
from functools import partial
import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import  Pose2D
import pygame as pg

from cairo_2d_sim.msg import Pose2DStamped
from cairo_2d_sim.lfd.state_space import Holonomic2DStateSpace, StateValidityChecker
from cairo_2d_sim.planning.constraints import UnconstrainedTSR, LineConstraintTSR
from cairo_2d_sim.planning.planners import CRRT, parametric_lerp

FILE_DIR = os.path.dirname(os.path.realpath(__file__))


if __name__ == '__main__':
    rospy.init_node("Planning publisher")
    
    state_space = Holonomic2DStateSpace()
    svc = StateValidityChecker()
    interp_fn = partial(parametric_lerp, steps=10)
    crrt = CRRT(state_space, svc, interp_fn, {'smooth_path': True, 'epsilon': .1, 'e_step': .25, 'smoothing_time': 10})
    
    start_q = [100, 100]
    goal_q = [1200, 800]
    
    tsr = UnconstrainedTSR()
    print(crrt.plan(tsr, start_q, goal_q))
    
    
    
    
    # state_pub = rospy.Publisher('/cairo_2d_sim/robot_state_replay', Pose2DStamped, queue_size=1)
    
    # filepath = os.path.join(FILE_DIR, "../data/test/execute_trajectory/test_execute_trajectory.json")
    # with open(filepath) as f:
    #     trajectory = json.load(f)
    
    # trajectory = [observation for observation in trajectory if observation['robot_state'] != {}]
    # prior_time = trajectory[0]['robot_state']['time']
    # for observation in trajectory:
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     pose2d = Pose2D()
    #     pose2d.x = observation['robot_state']['x']
    #     pose2d.y = observation['robot_state']['y']
    #     pose2d.theta = observation['robot_state']['theta']
    #     pose2dstamped = Pose2DStamped()
    #     pose2dstamped.header = header
    #     pose2dstamped.pose2d = pose2d
    #     state_pub.publish(pose2dstamped)
    #     rospy.sleep(observation['robot_state']['time'] - prior_time)
    #     prior_time = observation['robot_state']['time']
