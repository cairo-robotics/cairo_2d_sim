#! /usr/bin/env python3
import os
import json
import os
from pprint import pprint

from std_msgs.msg import Header
from geometry_msgs.msg import  Pose2D
import rospy

from cairo_2d_sim.planning.optimization import DualIntersectionOptimization, DualIntersectionWithTargetingOptimization, SingleIntersectionOptimization, SingleIntersectionWithTargetingOptimization
from cairo_2d_sim.msg import Pose2DStamped

from cairo_lfd.core.environment import SimpleObservation, Demonstration
from cairo_lfd.core.lfd import LfD2D
from cairo_lfd.data.alignment import DemonstrationAlignment
from cairo_lfd.data.labeling import DemonstrationLabler
from cairo_lfd.data.io import load_json_files

FILE_DIR = os.path.dirname(os.path.realpath(__file__))


def demo_vectorizor(demonstrations):
    vectorized_demos = []
    for demo in demonstrations:
        vectorized_demo = [[ob.data['robot']['x'], ob.data['robot']['y'], ob.data['robot']['theta']] for ob in demo]
        vectorized_demos.append(vectorized_demo)
    return vectorized_demos

# def observation_xy_vectorizor(ob):
#     return [ob.data['robot_state']['x'], ob.data['robot_state']['y']]

def observation_xytheta_vectorizor(ob):
    return [ob.data['robot']['x'], ob.data['robot']['y'], ob.data['robot']['theta']]



if __name__ == '__main__':
    rospy.init_node("optimization_node")

    
    test_keyframe_point_1 = (408, 120, 360)
    test_keyframe_point_2 = (408, 688, 360)
    test_keyframe_point_3 = (1180, 110, 180)
    first_intersection = (400, 100)
    second_intersection = (400, 700)
    third_intersection = (1200, 100)
    target = (805, 500)
    # constraint_1_2 = Constraint_1_2(first_intersection, second_intersection)
    # constraint_1_2.generate_model(test_keyframe_point)
    # description, val_x, val_y, val_a = constraint_1_2.solve()
    # print(description)
    # print(val_x, val_y)
    # print(val_a)
    print('keyframe point {} Constraint 1 & 2 with no targeting:'.format(test_keyframe_point_1))
    constraint_1_2 = DualIntersectionOptimization(first_intersection, second_intersection)
    constraint_1_2.solve(test_keyframe_point_1)
    
    print('keyframe point {} Constraint 1 & 2 with no targeting:'.format(test_keyframe_point_2))
    constraint_1_2 = DualIntersectionOptimization(first_intersection, second_intersection)
    constraint_1_2.solve(test_keyframe_point_2)
    
    print('keyframe point {} Constraint 1 & 2 with targeting:'.format(test_keyframe_point_1))
    constraint_1_2_targeting = DualIntersectionWithTargetingOptimization(first_intersection, second_intersection, target)
    constraint_1_2_targeting.solve(test_keyframe_point_1)
    
    print('keyframe point {} Constraint 1 & 2 with targeting:'.format(test_keyframe_point_2))
    constraint_1_2_targeting = DualIntersectionWithTargetingOptimization(first_intersection, second_intersection, target)
    constraint_1_2_targeting.solve(test_keyframe_point_2)
    
    print('keyframe point {} Constraint 2 & 3 with no targeting:'.format(test_keyframe_point_3))
    constraint_1_2_targeting = SingleIntersectionOptimization(third_intersection)
    constraint_1_2_targeting.solve(test_keyframe_point_3)
        
    print('keyframe point {} Constraint 2 & 3 with targeting:'.format(test_keyframe_point_3))
    constraint_1_2_targeting = SingleIntersectionWithTargetingOptimization(third_intersection, target)
    constraint_1_2_targeting.solve(test_keyframe_point_3)

