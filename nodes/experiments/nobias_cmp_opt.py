#! /usr/bin/env python3
import os
import json
import os
import itertools
from functools import partial
from pprint import pprint

from std_msgs.msg import Header, String
from geometry_msgs.msg import  Pose2D
import numpy as np
import rospy
import networkx as nx

from cairo_2d_sim.control.publish import publish_directed_point, publish_line, publish_text_label
from cairo_2d_sim.planning.distribution import KernelDensityDistribution
from cairo_2d_sim.planning.optimization import  DualIntersectionWithTargetingOptimization, SingleIntersectionWithTargetingOptimization
from cairo_2d_sim.planning.constraints import UnconstrainedTSR, LineTSR, DualLineTargetingTSR
from cairo_2d_sim.planning.state_space import Holonomic2DStateSpace, Holonomic2DBiasedStateSpace, StateValidityChecker
from cairo_2d_sim.planning.curve import xytheta_distance, parametric_xytheta_lerp, JointTrajectoryCurve
from cairo_2d_sim.planning.planners import CRRT, PlanningTimeoutException, MaxItersException
from cairo_2d_sim.evaluation.eval import IPDRelaxEvaluationTrial, IPDRelaxEvaluation
from cairo_2d_sim.msg import Pose2DStamped, MenuCommands

from cairo_lfd.core.environment import Observation, Demonstration
from cairo_lfd.core.lfd import LfD2D
from cairo_lfd.data.alignment import DemonstrationAlignment
from cairo_lfd.data.labeling import DemonstrationLabler
from cairo_lfd.data.io import load_json_files

FILE_DIR = os.path.dirname(os.path.realpath(__file__))


def demo_vectorizor(demonstrations):
    vectorized_demos = []
    for demo in demonstrations:
        vectorized_demo = [[ob.data["robot"]["x"], ob.data["robot"]["y"], ob.data["robot"]["theta"]] for ob in demo]
        vectorized_demos.append(vectorized_demo)
    return vectorized_demos

# def observation_xy_vectorizor(ob):
#     return [ob.data["robot_state"]["x"], ob.data["robot_state"]["y"]]

def observation_xytheta_vectorizor(ob):
    return [ob.data["robot"]["x"], ob.data["robot"]["y"], ob.data["robot"]["theta"]]

def extract_constraint_map_key(orderd_constraint_dict):
    constraint_key = []
    for key in orderd_constraint_dict.keys():
        if orderd_constraint_dict["c1"] is True:
            constraint_key.append(1)
        if orderd_constraint_dict["c2"] is True:
            constraint_key.append(2)
        if orderd_constraint_dict["c3"] is True:
            constraint_key.append(3)
    return tuple(set(constraint_key))


if __name__ == "__main__":
    
    ##############
    # ROS THINGS #
    ##############
    rospy.init_node("nobias_cmp_opt")
    circle_static_pub = rospy.Publisher("/cairo_2d_sim/create_directional_circle_static", String, queue_size=5)
    line_static_pub = rospy.Publisher("/cairo_2d_sim/create_line_static", String, queue_size=5)
    text_label_pub = rospy.Publisher("/cairo_2d_sim/create_text_label", String, queue_size=5)
    menu_commands_pub = rospy.Publisher('/cairo_2d_sim/menu_commands', MenuCommands, queue_size=5)
    state_pub = rospy.Publisher("/cairo_2d_sim/robot_state_replay", Pose2DStamped, queue_size=5)

    #########################
    # CONSTANTS / VARIABLES #
    #########################
    
    X_DOMAIN = [0, 1800]
    Y_DOMAIN = [0, 1000]
    THETA_DOMAIN = [0, 360]
    KEYFRAME_KDE_BANDWIDTH = .35
    SAMPLING_BIAS_KDE_BANDWIDTH = .1
    MOVE_TIME = 10
    EPSILON = 50
    EXTENSION_DISTANCE = 25
    MAX_SEGMENT_PLANNING_TIME = 60
    MAX_ITERS = 5000
    EVAL_OUTPUT_DIRECTORY = os.path.join(FILE_DIR, "../../data/experiments/bias_optfk/output")
    GOLD_DEMO_INPUT_DIRECTORY = os.path.join(FILE_DIR, "../../data/experiments/bias_optfk/input/gold/*.json")
    TRAINING_DEMO_INPUT_DIRECTORY = os.path.join(FILE_DIR, "../../data/experiments/bias_optfk/input/demos_1/*.json")
    TRIALS = 10
    
    ##############
    # EVALUATION #
    ##############
    
    # The evaluation object.
    evaluation = IPDRelaxEvaluation(EVAL_OUTPUT_DIRECTORY)
        
    # Create the gold demonstration trajectory
    gold_demo_data = load_json_files(GOLD_DEMO_INPUT_DIRECTORY)["data"][0]
    gold_demo_traj = [[entry["robot"]["x"], entry["robot"]["y"], entry["robot"]["theta"]] for entry in gold_demo_data]
    
    ##################################
    # TSR's for each line Constraint #
    ##################################
    # This will also be passed into the evaluation class.
    c2tsr_map = {}
    c2tsr_map[()] = UnconstrainedTSR()
    c2tsr_map[(1,)] = LineTSR((405, 105), (405, 805))
    c2tsr_map[(2,)] = DualLineTargetingTSR([405, 100], [1205, 100], [405, 700], [1205, 700], [805, 500])
    c2tsr_map[(3,)] = LineTSR((1205, 105), (1205, 505))

    ######################################
    # Constraint Intersection Optimizers #
    ######################################
    optimization_map = {}
    optimization_map[(1, 2)] = DualIntersectionWithTargetingOptimization((405, 105), (405, 705), (800, 500))
    optimization_map[(2, 3)] = SingleIntersectionWithTargetingOptimization((1205, 100), (800, 500))
    
    for _ in range(0, TRIALS):
        eval_trial = IPDRelaxEvaluationTrial()
        # Per trial evaluation data stores.
        TRAJECTORY_SEGMENTS = []
        EVAL_CONSTRAINT_ORDER = []
        IP_GEN_TIMES = []
        IP_GEN_TYPES = []
        PLANNING_FAILURE = False
        
        #################################################
        #         BUILD THE KEYFRAME MODEL              #
        #################################################
        # Import demonstration data and build the model #
        #################################################
        loaded_demonstration_data = load_json_files(TRAINING_DEMO_INPUT_DIRECTORY)
        # Convert imported data into Demonstrations and Observations
        demonstrations = []
        for datum in loaded_demonstration_data["data"]:
            observations = []
            for entry in datum:
                observations.append(Observation(entry))
            demonstrations.append(Demonstration(observations))
        if len(demonstrations) == 0:
            rospy.logwarn("No prior demonstration data to model!!")
            exit(0)
            
        label_settings = {
            "divisor": 20,
            "window": 10,
            "output": ""
        }
        
        alignment = DemonstrationAlignment(demo_vectorizor)
        demo_labeler = DemonstrationLabler(label_settings, alignment)
        labeled_initial_demos = demo_labeler.label(demonstrations)
        
        lfd = LfD2D(observation_xytheta_vectorizor)
        lfd.build_keyframe_graph(labeled_initial_demos, KEYFRAME_KDE_BANDWIDTH)

        ##############################################
        #          CREATE A PLANNING GRAPH          #
        #############################################
        # This will be used to generate a sequence  #
        # of constrained motion plans. The goal is  #
        # to create an IPD-Relaxed planning problem #   
        # Each end/start point in the graph will be #
        # a steering point in the Omega set of the  #
        # solution path.                            #
        #############################################
        planning_G = nx.Graph()
        
        # The start configuration. 
        start = (100, 500, 360)

        # Create a starting node for the planning graph.
        planning_G.add_nodes_from([("start", {"waypoint": start, "tsr": c2tsr_map[(1,)]})])
        EVAL_CONSTRAINT_ORDER.append((1,))
        
        prior_planning_G_vertex_id = 0
        for cur_node in lfd.G.get_keyframe_sequence():
            if lfd.G.nodes[cur_node]["keyframe_type"] == "constraint_transition": 
                planning_G.add_nodes_from([(int(cur_node))])
                # Get TSR/Optimizer for use in the segment start/endpoint loop
                # if no optimizer exists, we simply project according to the required constraint
                constraint_ids = extract_constraint_map_key(lfd.G.nodes[cur_node]["applied_constraints"])
                planning_G.nodes[int(cur_node)]["constraint_ids"] = constraint_ids

        # The goal configuration. 
        goal = (1205, 500, 360)
        
        # Create a goal node for the planning graph.
        planning_G.add_nodes_from([("goal", {"waypoint": goal})])
        
        
        planning_sequence = list(planning_G.nodes)
        # Here we use the planning order, creating a sequential pairing of ids to create edges.
        
        for edge in list(zip(planning_sequence, planning_sequence[1:])):
            planning_G.add_edge(edge[0], edge[1], tsr=c2tsr_map[(1,)])
        
        ####################################################
        #           SEQUENTIAL MANIFOLD PLANNING          #
        ####################################################
        # Now that we've defined our planning problem     #
        # withing a planning graph, which defines our SMP #
        # problem. We perform IPD relaxation and actual   #
        # planning.                                       #
        ####################################################

        full_trajectory = []
        
        # Here we use the keyframe planning order, creating a sequential pairing of keyframe ids.
        
        # Start overall planning time:
        eval_trial.start_timer("planning_time")
        try:
            for edge in list(zip(planning_sequence, planning_sequence[1:])):
                e1 = edge[0]
                e2 = edge[1]

                ######################################
                #  Unbiased state space for planning #
                ######################################
                planning_state_space = Holonomic2DStateSpace(X_DOMAIN, Y_DOMAIN, THETA_DOMAIN)
                planning_G.edges[edge]["planning_state_space"] = planning_state_space
                planning_G.edges[edge]['planning_tsr'] = c2tsr_map.get(planning_G.nodes[e1].get('constraint_ids', None), UnconstrainedTSR())
                EVAL_CONSTRAINT_ORDER.append(planning_G.nodes[e1].get("constraint_ids", None))

                # generate a starting point, and a steering point, according to constraints (if applicable). 
                # check if the starting point for the segment has generated already:
                if  planning_G.nodes[e1].get("waypoint", None) is None:
                    found = False
                    while not found:
                        eval_trial.start_timer("steering_point_generation_1")
                        ###############################################
                        # NO KEYFRAME SAMPLING FOR OPTIMIZATION!!!!!! #
                        # Instead we randomly sample from the BIASED  #
                        # state space                                 #
                        ###############################################
                        candidate_point = planning_G.edges[edge]["planning_state_space"].sample()
                        
                        # see if candidate point is already valid
                        tsr = c2tsr_map.get(planning_G.nodes[e1]["constraint_ids"], None)
                        if tsr is not None:
                            if tsr.validate(candidate_point):
                                waypoint = candidate_point
                                planning_G.nodes[e1]["waypoint"] = waypoint
                                found = True
                                IP_GEN_TYPES.append("direct")
                        
                        # get the TSR/Optimizer for use in the segment start/endpoint loop
                        point_optimizer = optimization_map.get(planning_G.nodes[e1]["constraint_ids"], None)
                        if point_optimizer is None and not found:
                            # we use the TSR projection to get the point
                            tsr = c2tsr_map.get(planning_G.nodes[e1]["constraint_ids"], None)
                            waypoint = tsr.project(candidate_point, None)
                            if waypoint is not None:
                                planning_G.nodes[e1]["waypoint"] = waypoint
                                found = True
                                IP_GEN_TYPES.append("projection")
                            else:
                                continue
                        elif not found:
                            print(point_optimizer)
                            waypoint = point_optimizer.solve(candidate_point)
                            if waypoint is not None:
                                planning_G.nodes[e1]["waypoint"] = waypoint
                                found = True
                                IP_GEN_TYPES.append("optimization")
                            else:
                                continue
                        IP_GEN_TIMES.append(eval_trial.end_timer("steering_point_generation_1"))
                        # Create a line between the two points. 
                        publish_line(line_static_pub, list(candidate_point[0:2]), waypoint[0:2], [0, 0, 0])
                        # Send to the game renderer:
                        publish_directed_point(circle_static_pub, list(candidate_point[0:2]), candidate_point[2], 8, [255, 25, 0])
                        # Send the label to the rendered:
                        publish_text_label(text_label_pub, [waypoint[0] + 5, waypoint[1] + 5], "Edge: {}, Constraints: {}".format(edge, planning_G.nodes[e1]["constraint_ids"]), [0, 0, 0])
                        # Send the updated correct point
                        publish_directed_point(circle_static_pub, waypoint[0:2], waypoint[2], 8, [0, 25, 255])
                        start = list(waypoint)
                else:
                    start = list(planning_G.nodes[e1]["waypoint"])
                if  planning_G.nodes[e2].get("waypoint", None) is None:
                    found = False
                    while not found:
                        eval_trial.start_timer("steering_point_generation_2")
                        ###############################################
                        # NO KEYFRAME SAMPLING FOR OPTIMIZATION!!!!!! #
                        # Instead we randomly sample from state space #
                        ###############################################
                        candidate_point = planning_G.edges[edge]["planning_state_space"].sample()
                        
                        # see if candidate point is already valid
                        tsr = c2tsr_map.get(planning_G.nodes[e2]["constraint_ids"], None)
                        if tsr is not None:
                            if tsr.validate(candidate_point):
                                waypoint = candidate_point
                                planning_G.nodes[e2]["waypoint"] = waypoint
                                found = True
                                IP_GEN_TYPES.append("direct")
                                
                        # get the Optimizer for use in the segment start/endpoint loop
                        point_optimizer = optimization_map.get(planning_G.nodes[e2]["constraint_ids"], None)
                        if point_optimizer is None and not found:
                            # we use the TSR projection to get the point if there is no optimizer to use.
                            waypoint = tsr.project(candidate_point, None)
                            if waypoint is not None:
                                planning_G.nodes[e2]["waypoint"] = waypoint
                                found = True
                                IP_GEN_TYPES.append("waypoint")
                            else:
                                continue
                        elif not found:
                            waypoint = point_optimizer.solve(candidate_point)
                            if waypoint is not None:
                                planning_G.nodes[e2]["waypoint"] = waypoint
                                found = True
                                IP_GEN_TYPES.append("optimization")
                            else:
                                continue
                        IP_GEN_TIMES.append(eval_trial.end_timer("steering_point_generation_2"))
                         # Create a line between the two points. 
                        publish_line(line_static_pub, list(candidate_point[0:2]), waypoint[0:2], [0, 0, 0])
                        # Send to the game renderer:
                        publish_directed_point(circle_static_pub, list(candidate_point[0:2]), candidate_point[2], 8, [255, 25, 0])
                         # Send the label to the rendered:
                        publish_text_label(text_label_pub, [waypoint[0] + 5, waypoint[1] + 5], "Edge: {}, Constraints: {}".format(edge, planning_G.nodes[e2]["constraint_ids"]), [0, 0, 0])
                        # Send the updated correct point
                        publish_directed_point(circle_static_pub, waypoint[0:2], waypoint[2], 8, [0, 25, 255])

                        end = list(waypoint)
                else:
                    end = list(planning_G.nodes[e2]["waypoint"])
                print(e1, e2)
                print(start, end)
                if e2 != "goal":
                    print(planning_G.nodes[e2]["constraint_ids"])
                print(planning_G.edges[edge]["planning_tsr"])
                print()
                
                # Constrained motion planning for specific manifold segment
                state_space = planning_G.edges[edge]["planning_state_space"]
                tsr = planning_G.edges[edge]["planning_tsr"]

                # state validity only checks if a poitn is epislong = 5 close to the start or goal. Too many points generated that close tostart and end creates cliques that the planner can not escape when plannign from start to end
                svc = StateValidityChecker(start, end)
                interp_fn = partial(parametric_xytheta_lerp, steps=10)

                
                planning_params = {
                    "smooth_path": False, 
                    "epsilon": EPSILON, 
                    "extension_distance": EXTENSION_DISTANCE,
                    "max_iters": MAX_ITERS,
                    "planning_timeout": MAX_SEGMENT_PLANNING_TIME
                }
                
                crrt = CRRT(state_space, svc, interp_fn, xytheta_distance, planning_params)
                
                plan = crrt.plan(tsr, start, end)
                
                if len(plan) == 0:
                    print("No initial plan found, retrying.")
                    crrt = CRRT(state_space, svc, interp_fn, xytheta_distance, planning_params)
                
                    plan = crrt.plan(tsr, start, end)
                if len(plan) == 0:
                    PLANNING_FAILURE = True
                    break
                path_points = crrt.get_path(plan)
                TRAJECTORY_SEGMENTS.append(path_points)
                full_trajectory = full_trajectory + path_points
        except PlanningTimeoutException:
            print("PLANNING TIMEOUT! PLANNING FAILURE!")
            PLANNING_FAILURE = True
            eval_trial.notes = "Planning timeout failure."
        except MaxItersException:
            print("MAX ITERS REACHED. PLANNING FAILURE!")
            PLANNING_FAILURE = True
            eval_trial.notes = "Planning max iters failure."

                
        if not PLANNING_FAILURE:
            eval_trial.planning_time = eval_trial.end_timer("planning_time")
            curve = JointTrajectoryCurve()
            timed_trajectory = curve.generate_trajectory([np.array(entry) for entry in full_trajectory], move_time=MOVE_TIME, num_intervals=1)
            trajectory = [point[1] for point in timed_trajectory]
            
            # Update trial evaluation data.
            eval_trial.path_length = eval_trial.eval_path_length(trajectory)
            eval_trial.success = eval_trial.eval_success(trajectory, goal, EPSILON)
            eval_trial.a2s_distance = eval_trial.eval_a2s([p[:2] for p in trajectory], [p[:2] for p in gold_demo_traj])
            eval_trial.a2f_percentage = eval_trial.eval_a2f(TRAJECTORY_SEGMENTS, c2tsr_map, EVAL_CONSTRAINT_ORDER)
            eval_trial.ip_gen_times = IP_GEN_TIMES
            eval_trial.ip_gen_types = IP_GEN_TYPES
            evaluation.add_trial(eval_trial)
            # Execute
            prior_time = timed_trajectory[0][0]
            for point in timed_trajectory:
                header = Header()
                header.stamp = rospy.Time.now()
                pose2d = Pose2D()
                pose2d.x = point[1][0]
                pose2d.y = point[1][1]
                pose2d.theta = point[1][2]
                pose2dstamped = Pose2DStamped()
                pose2dstamped.header = header
                pose2dstamped.pose2d = pose2d
                state_pub.publish(pose2dstamped)
                rospy.sleep(point[0] - prior_time)
                prior_time = point[0]
            mc = MenuCommands()
            mc.restart.data = True
            menu_commands_pub.publish(mc)
        else:
            # Update trial evaluation data with failure-style data.
            eval_trial.add_success("X")
            evaluation.add_trial(eval_trial)

            mc = MenuCommands()
            mc.restart.data = True
            menu_commands_pub.publish(mc)
           
    evaluation.export()
    mc = MenuCommands()
    mc.quit.data = True
    menu_commands_pub.publish(mc)
    exit()