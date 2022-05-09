#! /usr/bin/env python3
import os

import rospy

from cairo_lfd.core.environment import SimpleObservation, Demonstration
from cairo_lfd.data.alignment import DemonstrationAlignment
from cairo_lfd.data.labeling import DemonstrationLabler
from cairo_lfd.data.io import load_json_files

FILE_DIR = os.path.dirname(os.path.realpath(__file__))


def demo_vectorizor(demonstrations):
    vectorized_demos = []
    for demo in demonstrations:
        vectorized_demo = [[ob.data['robot_state']['x'], ob.data['robot_state']['y'], ob.data['robot_state']['theta']] for ob in demo]
        vectorized_demos.append(vectorized_demo)
    return vectorized_demos

if __name__ == '__main__':
    rospy.init_node("alignment_node", anonymous=True)
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
    print(filepath)
    loaded_demonstration_data = load_json_files(filepath)
    print(loaded_demonstration_data)
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
    
    # state_pub = rospy.Publisher('/cairo_2d_sim/robot_state_replay', Pose2DStamped, queue_size=1)
    
    # filepath = os.path.join(FILE_DIR, "../data/sessions/execute_trajectory/test_execute_trajectory.json")
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
