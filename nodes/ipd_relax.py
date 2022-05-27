#! /usr/bin/env python3
import os
import json
import os
import itertools
from functools import partial
from pprint import pprint

from std_msgs.msg import Header
from geometry_msgs.msg import  Pose2D
import rospy
import networkx as nx

from cairo_2d_sim.planning.distribution import KernelDensityDistribution
from cairo_2d_sim.planning.optimization import  DualIntersectionWithTargetingOptimization, SingleIntersectionWithTargetingOptimization
from cairo_2d_sim.planning.constraints import UnconstrainedPRMTSR, LineConstraintPRMTSR, LineTargetingConstraintPRMTSR
from cairo_2d_sim.planning.state_space import Holonomic2DStateSpace, Holonomic2DBiasedStateSpace, StateValidityChecker
from cairo_2d_sim.planning.curve import xytheta_distance, parametric_xytheta_lerp
from cairo_2d_sim.planning.planners import CPRM
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

def extract_constraint_map_key(orderd_constraint_dict):
    constraint_key = []
    for key in orderd_constraint_dict.keys():
        if orderd_constraint_dict['c1'] is True:
            constraint_key.append(1)
        if orderd_constraint_dict['c2'] is True:
            constraint_key.append(2)
        if orderd_constraint_dict['c3'] is True:
            constraint_key.append(3)
    return tuple(set(constraint_key))
            

if __name__ == '__main__':
    rospy.init_node("optimization_node")
        
    #############
    # CONSTANTS #
    #############
    
    X_DOMAIN = [0, 1800]
    Y_DOMAIN = [0, 1000]
    THETA_DOMAIN = [0, 360]
    

    MOVE_TIME = 5
    
    
    
    ##################################
    # TSR's for each line Constraint #
    ##################################
    c2tsr_map = {}
    c2tsr_map[()] = UnconstrainedPRMTSR()
    c2tsr_map[(1,)] = LineConstraintPRMTSR((405, 105), (405, 805))
    c2tsr_map[(2,)] = LineTargetingConstraintPRMTSR([405, 100], [1205, 100], [800, 500])
    c2tsr_map[(3,)] = LineConstraintPRMTSR((1195, 105), (1195, 505))

    ######################################
    # Constraint Intersection Optimizers #
    ######################################
    optimization_map = {}
    optimization_map[(1, 2)] = DualIntersectionWithTargetingOptimization((405, 105), (405, 805), (800, 500))
    optimization_map[(2, 3)] = SingleIntersectionWithTargetingOptimization((1205, 100), (800, 500))
    
    
    #################################################
    #         BUILD THE KEYFRAME MODEL              #
    #################################################
    # Import demonstration data and build the model #
    #################################################
    filepath = os.path.join(FILE_DIR, "../data/test/ipd_relax/*.json")
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
    
    # The intemediate trajectory data is used to bias planning of segments between cosntraint intersection points.
    intermediate_trajectories = {key: [[o.data for o in segment] for segment in group]
                                             for key, group in lfd.G.graph['intermediate_trajectories'].items()}
                
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
    # Create a goaL node for the planning graph.
    planning_G.add_nodes_from([("goal", {"waypoint": goal})])
    
    
    planning_sequence = list(planning_G.nodes)
    # Here we use the planning order, creating a sequential pairing of ids to create ecges.
    for edge in list(zip(planning_sequence, planning_sequence[1:])):
        planning_G.add_edge(edge[0], edge[1], tsr=c2tsr_map[(1,)])
       
    # ###################################################
    # #           SEQUENTIAL MANIFOLD PLANNING          #
    # ###################################################
    # # Now that we've defined our planning problem     #
    # # withing a planning graph, which defines our SMP #
    # # problem. We perform IPD relaxation and actual   #
    # # planning.                                       #
    # ###################################################
    

    full_trajectory = []
    
    # Here we use the keyframe planning order, creating a sequential pairing of keyframe ids.
    for edge in list(zip(planning_sequence, planning_sequence[1:])):
        e1 = edge[0]
        e2 = edge[1]

        # Generate the biasing state space for the edge:
        # create sampling biased State Space for use by the planner using the current keyframes upcoming segment intermediate trajectory data.
        if e1 == "start" or e1 == "goal":
            # We don't have intermediate trajectories from these points
            planning_state_space = Holonomic2DStateSpace(X_DOMAIN, Y_DOMAIN, THETA_DOMAIN)
        else:
            # The intermediate trajectories are index as the data the lead up to the node. So for edge e1 to e2, we use the e2 intermediate trajcewctory data. That is, the trajectory data that lead UP to e2.
            inter_trajs = intermediate_trajectories[int(e2)]
            inter_trajs_data = []
            for traj in inter_trajs:
                inter_trajs_data = inter_trajs_data + [[obsv['robot']['x'], obsv['robot']['y'], obsv['robot']['theta']]  for obsv in traj]
                inter_trajs_data.sort()
                inter_trajs_data = list(k for k, _ in itertools.groupby(inter_trajs_data))
            if len(inter_trajs_data) == 0:
                planning_state_space = Holonomic2DStateSpace(X_DOMAIN, Y_DOMAIN, THETA_DOMAIN)
            else:
                planning_biasing_distribution = KernelDensityDistribution(bandwidth=.25)
                planning_biasing_distribution.fit(inter_trajs_data)
                planning_state_space = Holonomic2DBiasedStateSpace(planning_biasing_distribution, X_DOMAIN, Y_DOMAIN, THETA_DOMAIN)
        planning_G.edges[edge]["planning_state_space"] = planning_state_space
        planning_G.edges[edge]['planning_tsr'] = c2tsr_map.get(planning_G.nodes[e1].get('constraint_ids', None), UnconstrainedPRMTSR())

    #     # generate a starting point, and a steering point, according to constraints (if applicable). 
    #     # check if the starting point for the segment has generated already:
        if  planning_G.nodes[e1].get('waypoint', None) is None:
            found = False
            while not found:
                # sample a candidate point from the keyframe model
                candidate_point = lfd.sample_from_keyframe(e1, n_samples=1)

                # get the TSR/Optimizer for use in the segment start/endpoint loop
                print(planning_G.nodes[e1]['constraint_ids'])
                point_optimizer = optimization_map.get(planning_G.nodes[e1]['constraint_ids'], None)
                if point_optimizer is None:
                    # we use the TSR projection to get the point
                    tsr = c2tsr_map.get(planning_G.nodes[e1]['constraint_ids'], None)
                    waypoint = tsr.project(candidate_point)
                    if waypoint is not None:
                        planning_G.nodes[e1]["waypoint"] = waypoint
                        found = True
                    else:
                        continue
                else:
                    waypoint = point_optimizer.solve(candidate_point)
                    if waypoint is not None:
                        planning_G.nodes[e1]["waypoint"] = waypoint
                        found = True
                    else:
                        continue
                start = list(waypoint)
        else:
            start = list(planning_G.nodes[e1]['waypoint'])
        if  planning_G.nodes[e2].get('waypoint', None) is None:
            found = False
            while not found:
                # sample a candidate point from the keyframe model
                candidate_point = lfd.sample_from_keyframe(e2, n_samples=1)
                
                # get the TSR/Optimizer for use in the segment start/endpoint loop
                print(planning_G.nodes[e2]['constraint_ids'])
                point_optimizer = optimization_map.get(planning_G.nodes[e2]['constraint_ids'], None)
                if point_optimizer is None:
                    # we use the TSR projection to get the point
                    tsr = c2tsr_map.get(planning_G.nodes[e2]['constraint_ids'], None)
                    waypoint = tsr.project(candidate_point)
                    if waypoint is not None:
                        planning_G.nodes[e2]["waypoint"] = waypoint
                        found = True
                    else:
                        continue
                else:
                    waypoint = point_optimizer.solve(candidate_point)
                    if waypoint is not None:
                        planning_G.nodes[e2]["waypoint"] = waypoint
                        found = True
                    else:
                        continue
                end = list(waypoint)
        else:
            end = list(planning_G.nodes[e2]['waypoint'])
        print("\n\nSTART AND END")
        print(e1, e2)
        print(start, end)
        print(planning_G.nodes[e2]['constraint_ids'])
        print(planning_G.edges[edge]['planning_tsr'])
        print()
        
        # Constrained motion planning for specific manifold segment
        state_space = planning_G.edges[edge]['planning_state_space']
        tsr = planning_G.edges[edge]['planning_tsr']
        print(tsr)
        # state validity only checks if a poitn is epislong = 5 close to the start or goal. Too many points generated that close tostart and end creates cliques that the planner can not escape when plannign from start to end
        svc = StateValidityChecker(start, end)
        interp_fn = partial(parametric_xytheta_lerp, steps=10)
        prm = CPRM(state_space, svc, interp_fn, xytheta_distance, {'smooth_path': False, 'ball_radius': 100, 'n_samples': 500, 'k': 100})
        
        plan = prm.plan(tsr, start, end)
        
        if len(plan) == 0:
            print("No initial plan found, ramping up number of points")
            prm = CPRM(state_space, svc, interp_fn, xytheta_distance, {'smooth_path': False, 'ball_radius': 100, 'n_samples': 2000, 'k': 100})
        
            plan = prm.plan(tsr, start, end)
        if len(plan) == 0:
            raise Exception("One segment failed to find a plan")
        path_points = prm.get_path(plan)
    
        full_trajectory = full_trajectory + path_points
        
    timed_trajectory = list(zip([MOVE_TIME * n/len(full_trajectory) for n in range(0, len(full_trajectory))], [q[0] for q in full_trajectory], [q[1] for q in full_trajectory], [q[2] for q in full_trajectory]))
    
    # Execute
        
    state_pub = rospy.Publisher('/cairo_2d_sim/robot_state_replay', Pose2DStamped, queue_size=1)
    
    prior_time = timed_trajectory[0][0]
    for point in timed_trajectory:
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
