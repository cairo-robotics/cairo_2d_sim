#!/usr/bin/env python3
import os

import rospy

from cairo_2d_sim.evaluation.analysis import PlanningTimeAnalysis, SuccessPercentageAnalysis, PathLengthAnalysis, A2SAnalysis, A2FAnalysis

FILE_DIR = os.path.dirname(os.path.realpath(__file__))

if __name__ == "__main__":
    
    ##############
    # ROS THINGS #
    ##############
    rospy.init_node("analysis")
    
    #########################
    # CONSTANTS / VARIABLES #
    #########################
    EVAL_OUTPUT_DIRECTORY = os.path.join(FILE_DIR, "../../data/experiments/output_combined")
    
    ####################
    # Analysis Classes #
    ####################
    print()
    pta = PlanningTimeAnalysis(EVAL_OUTPUT_DIRECTORY)
    print(pta.stats())
    spa = SuccessPercentageAnalysis(EVAL_OUTPUT_DIRECTORY)
    print(spa.stats())
    print()
    pla = PathLengthAnalysis(EVAL_OUTPUT_DIRECTORY)
    print(pla.stats())
    print()
    a2sa = A2SAnalysis(EVAL_OUTPUT_DIRECTORY)
    print(a2sa.stats())
    print()
    a2fa = A2FAnalysis(EVAL_OUTPUT_DIRECTORY)
    print(a2fa.stats())