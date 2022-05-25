#! /usr/bin/env python3
import os
import json
import os
from functools import partial
from pprint import pprint

from std_msgs.msg import Header
from geometry_msgs.msg import  Pose2D
import rospy
import networkx as nx

from cairo_2d_sim.planning.distribution import KernelDensityDistribution
from cairo_2d_sim.planning.optimization import  DualIntersectionOptimization, SingleIntersection
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
    

    
    
    
    
    ##################################
    # TSR's for each line Constraint #
    ##################################
    c2tsr_map = {}
    c2tsr_map[()] = UnconstrainedPRMTSR()
    c2tsr_map[(1,)] = LineConstraintPRMTSR((405, 105), (405, 805))
    c2tsr_map[(2,)] = LineTargetingConstraintPRMTSR([405, 100], [1205, 100], [800, 500])
    c2tsr_map[(3,)] = LineConstraintPRMTSR((1195, 105), (1595, 105))

    ######################################
    # Constraint Intersection Optimizers #
    ######################################
    optimization_map = {}
    optimization_map[(1, 2)] = DualIntersectionOptimization((405, 105), (405, 805))
    optimization_map[(2, 3)] = SingleIntersection((1205, 100))
    
    
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
    start = (405, 500, 360)
    # Create a starting node for the planning graph.
    planning_G.add_nodes_from([("start", {"waypoint": start, "tsr": c2tsr_map[(1,)]})])
    prior_planning_G_vertex_id = 0
    for cur_node in lfd.G.get_keyframe_sequence():
        if lfd.G.nodes[cur_node]["keyframe_type"] == "constraint_transition": 
            planning_G.add_nodes_from([(int(cur_node))])
            # create sampling biased State Space for use by the planner using the current keyframes upcoming segment intermediate trajectory data.
            inter_trajs = intermediate_trajectories[int(cur_node)]
            inter_trajs_data = []
            for traj in inter_trajs:
                inter_trajs_data = inter_trajs_data + [[obsv['robot']['x'], obsv['robot']['y'], obsv['robot']['theta']]  for obsv in traj]
            if len(inter_trajs_data) == 0:
                state_space = Holonomic2DStateSpace(X_DOMAIN, Y_DOMAIN, THETA_DOMAIN)
            else:
                planning_biasing_distribution = KernelDensityDistribution()
                planning_biasing_distribution.fit(inter_trajs_data)
                state_space = Holonomic2DBiasedStateSpace(planning_biasing_distribution, X_DOMAIN, Y_DOMAIN, THETA_DOMAIN)
            planning_G.nodes[int(cur_node)]["state_space"] = state_space
            # Get TSR/Optimizer for use in the segment start/endpoint loop
            # if no optimizer exists, we simply project according to the required constraint
            constraint_ids = extract_constraint_map_key(lfd.G.nodes[cur_node]["applied_constraints"])
            planning_G.nodes[int(cur_node)]["constraint_ids"] = constraint_ids

            
    # The goal configuration. 
    goal = [(405, 500, 360)]
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
    

    
    
    # Here we use the keyframe planning order, creating a sequential pairing of keyframe ids.
    for edge in list(zip(planning_sequence, planning_sequence[1:])):
        e1 = edge[0]
        e2 = edge[1]
        # Get the TSR/Optimizer for use in the segment start/endpoint loo
    #     # Create the TSR object
    #     planning_tsr_config =  planning_G.nodes[e1].get("tsr", unconstrained_TSR)
    #     T0_w = xyzrpy2trans(planning_tsr_config['T0_w'], degrees=planning_tsr_config['degrees'])
    #     Tw_e = xyzrpy2trans(planning_tsr_config['Tw_e'], degrees=planning_tsr_config['degrees'])
    #     Bw = bounds_matrix(planning_tsr_config['Bw'][0], planning_tsr_config['Bw'][1])
    #     # we plan with the current edges first/starting node's tsr and planning space.
    #     planning_tsr = TSR(T0_w=T0_w, Tw_e=Tw_e, Bw=Bw, bodyandlink=0, manipindex=16)
    #     keyframe_space_e1 = planning_G.nodes[e1]['keyframe_space']
        
    #     # generate a starting point, and a steering point, according to constraints (if applicable). 
    #     # check if the starting point for the segment has generated already:
        if  planning_G.nodes[e1].get('waypoint', None) is None:
            found = False
            while not found:
                planning_G.nodes[e1]
                # get the TSR/Optimizer for use in the segment start/endpoint loop
                point_optimizer = optimization_map.get(planning_G.nodes[e1]['constraint_ids'], None)
                if point_optimizer is None:
                    # we use the TSR projection to get the point
                    tsr = c2tsr_map.get(planning_G.nodes[e1]['constraint_ids'], None)
                    waypoint = tsr.project(lfd.sample)
                else:
                    
    #             found = False
    #             while not found:
    #                 raw_sample = keyframe_space_e1.sample()
    #                 sample = []
    #                 for value in raw_sample:
    #                     sample.append(wrap_to_interval(value))
    #                 # If the sample is already constraint compliant, no need to project. Thanks LfD!
    #                 err, _ = distance_to_TSR_config(sawyer_robot, sample, planning_tsr)
    #                 if err < .1 and svc.validate(sample):
    #                     start = sample
    #                     planning_G.nodes[e1]['point'] = start
    #                     found = True
    #                 elif svc.validate(sample):
                       
    #                     q_constrained = project_config(sawyer_robot, planning_tsr, np.array(
    #                     sample), np.array(sample), epsilon=.025, e_step=.35, q_step=100)
    #                     normalized_q_constrained = []
    #                     # If there is a foliation model, then we must perform rejection sampling until the projected sample is classified 
    #                     # to the node's foliation value
    #                     if q_constrained is not None:
    #                         if foliation_model is not None:
    #                             # This is the rejection sampling step to enforce the foliation choice
    #                             if foliation_model.predict(np.array([q_constrained])) == foliation_value:
    #                                 for value in q_constrained:
    #                                     normalized_q_constrained.append(
    #                                         wrap_to_interval(value))
    #                             else:
    #                                 continue
    #                         else:
    #                             for value in q_constrained:
    #                                 normalized_q_constrained.append(
    #                                     wrap_to_interval(value))
    #                     else:
    #                         continue
    #                     if svc.validate(normalized_q_constrained):
    #                         start = normalized_q_constrained
    #                         # We've generated a point so lets use it moving forward for all other planning segments. 
    #                         planning_G.nodes[e1]['point'] = start
    #                         found = True
    #     # if the point steering/start point is available already, then we simply use it 
    #     else:
    #         start = planning_G.nodes[e1]['point']
    #         print(start)

    #     if e2 == 33 or e2 == '33':
    #         print("Made it")

    #     if  planning_G.nodes[e2].get('point', None) is None:
    #         keyframe_space_e2 =  planning_G.nodes[e2]['keyframe_space']
    #         tsr_config =  planning_G.nodes[e2].get("tsr", unconstrained_TSR)
    #         T0_w2 = xyzrpy2trans(tsr_config['T0_w'], degrees=tsr_config['degrees'])
    #         Tw_e2 = xyzrpy2trans(tsr_config['Tw_e'], degrees=tsr_config['degrees'])
    #         Bw2 = bounds_matrix(tsr_config['Bw'][0], tsr_config['Bw'][1])
    #         tsr = TSR(T0_w=T0_w2, Tw_e=Tw_e2, Bw=Bw2, bodyandlink=0, manipindex=16)
            
    #         print("Constraints {}: {}".format(e2, planning_G.nodes[e2].get("constraint_ids", [])))
    #         foliation_model =  planning_G.nodes[e2].get("foliation_model", None)
    #         foliation_value =  planning_G.nodes[e2].get("foliation_value", None)

    #         with DisabledCollisionsContext(sim, [], [], disable_visualization=True):
    #             found = False
    #             while not found:
    #                 raw_sample = keyframe_space_e2.sample()
    #                 sample = []
    #                 for value in raw_sample:
    #                     sample.append(wrap_to_interval(value))
    #                 # If the sample is already constraint compliant, no need to project. Thanks LfD!
    #                 # print(sawyer_robot.solve_forward_kinematics(sample)[0][0], quat2rpy(sawyer_robot.solve_forward_kinematics(sample)[0][1]))
    #                 err, _ = distance_to_TSR_config(sawyer_robot, sample, tsr)
    #                 # print(err)
    #                 if err < .1 and svc.validate(sample):
    #                     end = sample
    #                     planning_G.nodes[e2]['point'] = end
    #                     found = True
    #                 elif svc.validate(sample):
    #                     # We use a large q_step since we're not using project_config for cbirrt2 but instead just trying to project as ingle point. We don't care about how far we're stepping w.r.t tree growth
    #                     q_constrained = project_config(sawyer_robot, tsr, np.array(
    #                     sample), np.array(sample), epsilon=.025, e_step=.25, q_step=100)
    #                     normalized_q_constrained = []
    #                     if q_constrained is not None:
    #                         if foliation_model is not None:
    #                             # This is the rejection sampling step to enforce the foliation choice
    #                             if foliation_model.predict(np.array([q_constrained])) == foliation_value:
    #                                 for value in q_constrained:
    #                                     normalized_q_constrained.append(
    #                                         wrap_to_interval(value))
    #                             else:
    #                                 continue
    #                         else:
    #                             for value in q_constrained:
    #                                 normalized_q_constrained.append(
    #                                     wrap_to_interval(value))
    #                     else:
    #                         continue
    #                     if svc.validate(normalized_q_constrained):
    #                         err, _ = distance_to_TSR_config(sawyer_robot, normalized_q_constrained, tsr)
    #                         print(err)
    #                         end = normalized_q_constrained
    #                         # We've generated a point so lets use it moving forward for all other planning segments. 
    #                         planning_G.nodes[e2]['point'] = end
    #                         found = True
    #     else:
    #         end = planning_G.nodes[e2]['point']
    #     print("\n\nSTART AND END\n")
    #     print(start, end)
    #     print(np.linalg.norm(np.array(start) - np.array(end)))
    #     print("\n\n")
    #     if np.linalg.norm(np.array(start) - np.array(end)) > .1:
    #         with DisabledCollisionsContext(sim, [], [], disable_visualization=True):
    #             ###########
    #             # CBiRRT2 #
    #             ###########
    #             # Use parametric linear interpolation with 10 steps between points.
    #             interp = partial(parametric_lerp, steps=10)
    #             # See params for CBiRRT2 specific parameters 
    #             cbirrt = CBiRRT2(sawyer_robot, planning_state_space, svc, interp, params={'smooth_path': True, 'smoothing_time': 5, 'epsilon': .08, 'q_step': .38, 'e_step': .25, 'iters': 20000})
    #             logger.info("Planning....")
    #             print("Start, end: ", start, end)
    #             logger.info("Constraints: {}".format(planning_G.nodes[e1].get('constraint_ids', None)))
    #             print(planning_tsr_config)
    #             plan = cbirrt.plan(planning_tsr, np.array(start), np.array(end))
    #             path = cbirrt.get_path(plan)
    #         if len(path) == 0:
    #             logger.info("Planning failed....")
    #             sys.exit(1)
    #         logger.info("Plan found....")
    #     else:
    #         # sometimes the start point is really, really close to the a keyframe so we just inerpolate, since really close points are challenging the CBiRRT2 given the growth parameters
    #         path = [list(val) for val in interp_fn(np.array(start), np.array(end))]

    #     # splining uses numpy so needs to be converted
    #     logger.info("Length of path: {}".format(len(path)))
    #     final_path = final_path + path
    #     sim_context.delete_context()
               
   
    # sim_context = SawyerBiasedSimContext(config, setup=False)
    # sim_context.setup(sim_overrides={"use_gui": True, "run_parallel": False})
    # sim = sim_context.get_sim_instance()
    # logger = sim_context.get_logger()
    # sawyer_robot = sim_context.get_robot()
    # svc = sim_context.get_state_validity()
    # interp_fn = partial(parametric_lerp, steps=10)
    # sawyer_robot.set_joint_state(start_configuration)
    # key = input("Press any key to excute plan.")

    # # splining uses numpy so needs to be converted
    # planning_path = [np.array(p) for p in final_path]
    # # Create a MinJerk spline trajectory using JointTrajectoryCurve and execute
    # jtc = JointTrajectoryCurve()
    # traj = jtc.generate_trajectory(planning_path, move_time=20)
    # try:
    #     prior_time = 0
    #     for i, point in enumerate(traj):
    #         if not svc.validate(point[1]):
    #             print("Invalid point: {}".format(point[1]))
    #             continue
    #         sawyer_robot.set_joint_state(point[1])
    #         time.sleep(point[0] - prior_time)
    #         prior_time = point[0]
    # except KeyboardInterrupt:
    #     exit(1)