#! /usr/bin/env python3
import os
import json
import os
from pprint import pprint

from std_msgs.msg import Header
from geometry_msgs.msg import  Pose2D
import rospy

from cairo_2d_sim.lfd.optimization import Constraint_1_2
from cairo_2d_sim.msg import Pose2DStamped

from cairo_lfd.core.environment import SimpleObservation, Demonstration
from cairo_lfd.core.lfd import LfD2D
from cairo_lfd.data.alignment import DemonstrationAlignment
from cairo_lfd.data.labeling import DemonstrationLabler
from cairo_lfd.data.io import load_json_files
from cairo_2d_sim.lfd.optimization import Constraint_1_2, Constraint_1_2_gekko
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
    # arg_fmt = argparse.RawDescriptionHelpFormatter
    # parser = argparse.ArgumentParser(formatter_class=arg_fmt,
    #                                  description=main.__doc__)
    # required = parser.add_argument_group('required arguments')

    # required.add_argument(
    #     '-i', '--input_directory', dest='input_directory', required=False,
    #     help='the directory from which to input prior demonstration .json files'
    # )

    # args = parser.parse_args(rospy.myargv()[1:])
    

    # loaded_demonstration_data = load_json_files(args.input_directory + "/*.json")
    filepath = os.path.join(FILE_DIR, "../data/test/alignment/*.json")
    loaded_demonstration_data = load_json_files(filepath)
    # Convert imported data into Demonstrations and Observations
    demonstrations = []
    for datum in loaded_demonstration_data["data"]:
        observations = []
        for entry in datum:
            observations.append(SimpleObservation(entry))
        demonstrations.append(Demonstration(observations))
    if len(demonstrations) == 0:
        rospy.logwarn("No prior demonstration data to model!!")
        exit(0)
        
    label_settings = {
        "divisor": 20,
        "window": 10,
        "output": ""
    }
     
    print(demonstrations[0].observations[0].data)
    alignment = DemonstrationAlignment(demo_vectorizor)
    demo_labeler = DemonstrationLabler(label_settings, alignment)
    labeled_initial_demos = demo_labeler.label(demonstrations)
    print(labeled_initial_demos)
    
    lfd = LfD2D(observation_xytheta_vectorizor)
    lfd.build_keyframe_graph(labeled_initial_demos, .05)
    lfd.sample_keyframes(1)
    waypoint_pairs = lfd.get_model_waypoints()
    pprint(waypoint_pairs)
    
    test_keyframe_point = (408, 720)
    first_intersection = (400, 100)
    second_intersection = (400, 700)
    # constraint_1_2 = Constraint_1_2(first_intersection, second_intersection)
    # constraint_1_2.generate_model(test_keyframe_point)
    # description, val_x, val_y, val_a = constraint_1_2.solve()
    # print(description)
    # print(val_x, val_y)
    # print(val_a)
    
    constraint_1_2 = Constraint_1_2_gekko(first_intersection, second_intersection)
    constraint_1_2.solve(test_keyframe_point)
