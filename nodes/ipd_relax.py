#! /usr/bin/env python3
import os
import json
import os
from pprint import pprint

from std_msgs.msg import Header
from geometry_msgs.msg import  Pose2D
import rospy
import networkx as nx

from cairo_2d_sim.lfd.optimization import  DualIntersectionOptimization, SingleIntersection
from cairo_2d_sim.planning.constraints import UnconstrainedPRMTSR, LineConstraintPRMTSR, LineTargetingConstraintPRMTSR
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
    keyframe_data = {'keyframes': {}}
    for cur_node in lfd.G.get_keyframe_sequence():
            keyframe_data['keyframes'][cur_node] = {}
            keyframe_data['keyframes'][cur_node]['applied_constraints'] = lfd.G.nodes[cur_node]["applied_constraints"]
            keyframe_data['keyframes'][cur_node]['observations'] = [
                obsv.data for obsv in lfd.G.nodes[cur_node]["observations"]]
            keyframe_data['keyframes'][cur_node]['keyframe_type'] = lfd.G.nodes[cur_node]["keyframe_type"]
    for keyframe_id in keyframe_data['keyframes']:
        if keyframe_data['keyframes'][keyframe_id]["keyframe_type"] == "constraint_transition":
            pprint(keyframe_data['keyframes'][keyframe_id]['applied_constraints'])
            
            # Map applied constraint to TSR
            # if the transition has a known intersection / more than one constraint use the optimization, else default to sampling / steering point a keyframe point by projecting using the associated TSR
            # make biasing data from observations
            
            
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
    start = [(405, 500, 360)]


    # # Starting and ending keyframe ids
    # start_keyframe_id = list(keyframes.keys())[0]
    # end_keyframe_id = list(keyframes.keys())[-1]

    # ###############################################################################
    # # Insert last keyframe into planning graph before looping over keyframe model #
    # ###############################################################################
    
    # # We will build a keyframe dsitribution using KDE from which to sample for steering points / viapoints. 
    # end_data = [obsv['robot']['joint_angle'] for obsv in keyframes[end_keyframe_id]["observations"]]
    # keyframe_dist = KernelDensityDistribution(bandwidth=.05)
    # keyframe_dist.fit(end_data)
    # keyframe_space = DistributionSpace(sampler=DistributionSampler(keyframe_dist, fraction_uniform=0), limits=limits)
    # # we cast the keyframe ids to int for networkx node dereferencing as keyframe ids are output as strings from CAIRO LfD 
    # planning_G.add_nodes_from([int(end_keyframe_id)], keyframe_space=keyframe_space)

    # # get the constraint IDs
    # constraint_ids = keyframes[end_keyframe_id]["applied_constraints"]
    # foliation_constraint_ids = list(set(keyframes[end_keyframe_id]["applied_constraints"] + keyframes[list(keyframes.keys())[-2]]["applied_constraints"]))
    # planning_G.nodes[int(end_keyframe_id)]["constraint_ids"] = constraint_ids
    
    # # get the foliation model
    # foliation_model = c2f_map.get(tuple(sorted(foliation_constraint_ids)), None)
    
    # # there's a possibility that the ending keyframe is not constrained and thus might not provide a foliation model to use
    # if foliation_model is not None:
    #     planning_G.nodes[int(end_keyframe_id)]["foliation_model"] = foliation_model

    #     # Determine the foliation choice based on the keyframe data and assign in to the planning graph node
    #     # Winner takes all essentially chooses the most common foliation value base on classifying each data point
    #     foliation_value = winner_takes_all(data, foliation_model)
    #     upcoming_foliation_value = foliation_value
    
    #     planning_G.nodes[int(end_keyframe_id)]["foliation_value"] = foliation_value
    # else:
    #     upcoming_foliation_value = None

    # # Get the TSR configurations so they can be appended to the  associated with constraint ID combo.
    # planning_G.nodes[int(end_keyframe_id)]['tsr'] = c2tsr_map.get(tuple(sorted(constraint_ids)), unconstrained_TSR)
    
    # # the end id will be the first upcoming ID
    # upcoming_id = int(end_keyframe_id)
        
        
    # ############################################################################
    # # Reverse iteration over the keyframe model to populate our planning graph #
    # ############################################################################
    
    # reversed_keyframes = list(reversed(keyframes.items()))[1:]
    
    # # used to keep track of sequence of constraint transition, start, and end keyframe ids as
    # # not all keyframes in the lfd model will be used
    # keyframe_planning_order = []
    # keyframe_planning_order.insert(0, int(end_keyframe_id))
    
    # # iteration over the list of keyframes in reverse order ecluding the last keyframe ID which has been handled above
    # for idx, item in enumerate(reversed_keyframes):
    #     keyframe_id = int(item[0])
    #     keyframe_data = item[1]
    #     # We only use constraint transition, start, and end keyframes (which has already been accounted for above).
    #     if keyframe_data["keyframe_type"] == "constraint_transition" or keyframe_id == int(start_keyframe_id):
            
    #         # We keep track of the sequence of keyframe ids in order to established a static ordering of keyframe pairs to plan between
    #         keyframe_planning_order.insert(0, keyframe_id)

    #         # Copy the base planning config. This will be updated with specfic configurations for this planning segment (tsrs, biasing etc,.)
    #         planning_config = copy.deepcopy(base_config)

    #         # Create KDE distrubtion for the current keyframe.
    #         data = [obsv['robot']['joint_angle'] for obsv in keyframe_data["observations"]]
    #         keyframe_dist = KernelDensityDistribution(bandwidth=.05)
    #         keyframe_dist.fit(data)
    #         # We want to fully bias sampling from keyframe distributions.
    #         keyframe_space = DistributionSpace(sampler=DistributionSampler(keyframe_dist, fraction_uniform=0), limits=limits)

    #         # Let's create the node and add teh keyframe KDE model as a planning space.
    #         planning_G.add_nodes_from([keyframe_id], keyframe_space=keyframe_space)

    #         # get the constraint IDs
    #         constraint_ids = keyframe_data["applied_constraints"]
            
    #         # The foliation constraint ids combines both start and end keyframes of the planning segment. In other words, we need to 
    #         # ensure the start point and ending steering point are in the same foliation, so we utilize the constraints from both keyframes.
    #         foliation_constraint_ids = list(set(keyframe_data["applied_constraints"] + keyframes[str(upcoming_id)]["applied_constraints"]))

    #         planning_G.nodes[keyframe_id]["constraint_ids"] = constraint_ids
            
    #         # Get the TSR configurations so they can be appended to both the keyframe and the edge between associated with constraint ID combo.
    #         planning_G.nodes[keyframe_id]['tsr'] = c2tsr_map.get(tuple(sorted(constraint_ids)), unconstrained_TSR)
    #         planning_config['tsr'] = c2tsr_map.get(tuple(sorted(constraint_ids)), unconstrained_TSR)
            
    #         # get the foliation model
    #         foliation_model = c2f_map.get(tuple(sorted(foliation_constraint_ids)), None)
    #         if foliation_model is not None:
    #             # Assign the foliation model to the planning graph node for the current keyframe.
    #             planning_G.nodes[keyframe_id]["foliation_model"] = foliation_model
    #             # Determine the foliation choice based on the keyframe data and assign in to the Pg node
    #             foliation_value = winner_takes_all(data, foliation_model)
    #             # We want the current foliation value / component to be equivalent to the upcoming foliation value
    #             # TODO: Integrate equivalency set information so that based on trajectories, we have some confidence if to 
    #             # foliation values are actually from the same foliation value. 
    #             if foliation_value != upcoming_foliation_value and upcoming_foliation_value is not None:
    #                 print("Foliation values are not equivalent, cannot gaurantee planning feasibility but will proceed.")
    #             planning_G.nodes[keyframe_id]["foliation_value"] = foliation_value
    #             upcoming_foliation_value = foliation_value
    #         # If the current keyframe doesn't have a constraint with an associated model then we dont care to hold 
    #         # onto the upcoming foliation value any longer
    #         else:
    #             upcoming_foliation_value = None

            
    #         if keyframe_id != int(start_keyframe_id):
    #             # Create intermediate trajectory ditribution configuration.
    #             inter_trajs = intermediate_trajectories[str(keyframe_id)]
    #             inter_trajs_data = []
    #             for traj in inter_trajs:
    #                 inter_trajs_data = inter_trajs_data + [obsv['robot']['joint_angle'] for obsv in traj]
                
    #             # this information will be used to create a biasing distribution for sampling during planning between steering points.
    #             sampling_bias = {
    #                 'bandwidth': .1,
    #                 'fraction_uniform': .1,
    #                 'data': inter_trajs_data
    #             }
    #             planning_config['sampling_bias'] = sampling_bias

    #         planning_G.add_edge(keyframe_id, upcoming_id)
    #         # Finally add the planning config to the planning graph edge. 
    #         planning_G.edges[keyframe_id, upcoming_id]['config'] = planning_config

    #         # update the upcoming keyframe id with the current id
    #         upcoming_id = keyframe_id
 

    # # Let's insert the starting point:
    # # Copy the base planning config. This will be updated with specfic configurations for this planning segment (tsrs, biasing etc,.)
    # planning_config = copy.deepcopy(base_config)
    # # We populat ethe "point" attribute of the planning graph node which will indicate that we do not need to sample from this node
    # # We also use a basic keyframe space -> TODO: is this necessary?
    # planning_G.add_nodes_from([(0, {"point": start_configuration, "keyframe_space": SawyerConfigurationSpace(limits=limits)})])
    # planning_G.nodes[0]['tsr'] = TSR_1_config
    # # let's connect the starting point to the node associated with the starting keyframe
    # planning_G.add_edge(0, int(start_keyframe_id))
    # keyframe_planning_order.insert(0, 0)
    # planning_config['tsr'] = TSR_1_config
    # planning_G.nodes[int(start_keyframe_id)]['tsr'] = TSR_1_config
    # # Add the lanning config to the planning graph edge. 
    # planning_G.edges[0, int(start_keyframe_id)]['config'] = planning_config
    # # A list to append path segments in order to create one continuous path
    # final_path = []
    
    # ###################################################
    # #           SEQUENTIAL MANIFOLD PLANNING          #
    # ###################################################
    # # Now that we've defined our planning problem     #
    # # withing a planning graph, which defines our SMP #
    # # problem. We perform IPD relaxation and actual   #
    # # planning.                                       #
    # ###################################################
    
    # # Here we use the keyframe planning order, creating a sequential pairing of keyframe ids.
    # for edge in list(zip(keyframe_planning_order, keyframe_planning_order[1:])):
    #     e1 = edge[0]
    #     e2 = edge[1]
    #     edge_data = planning_G.edges[e1, e2]
    #     # lets ge the planning config from the edge or use the generic base config defined above
    #     config = edge_data.get('config', base_config)
        
    #     # We create a Sim context from the config for planning. 
    #     sim_context = SawyerBiasedSimContext(config, setup=False)
    #     sim_context.setup(sim_overrides={"use_gui": False, "run_parallel": False})
    #     planning_state_space = sim_context.get_state_space() # The biased state space for sampling points according to intermediate trajectories.
    #     sim = sim_context.get_sim_instance()
    #     logger = sim_context.get_logger()
    #     sawyer_robot = sim_context.get_robot()
    #     svc = sim_context.get_state_validity() # the SVC is the same for all contexts so we will use this one in our planner.
    #     interp_fn = partial(parametric_lerp, steps=10)

    #     # Create the TSR object
    #     planning_tsr_config =  planning_G.nodes[e1].get("tsr", unconstrained_TSR)
    #     T0_w = xyzrpy2trans(planning_tsr_config['T0_w'], degrees=planning_tsr_config['degrees'])
    #     Tw_e = xyzrpy2trans(planning_tsr_config['Tw_e'], degrees=planning_tsr_config['degrees'])
    #     Bw = bounds_matrix(planning_tsr_config['Bw'][0], planning_tsr_config['Bw'][1])
    #     # we plan with the current edges first/starting node's tsr and planning space.
    #     planning_tsr = TSR(T0_w=T0_w, Tw_e=Tw_e, Bw=Bw, bodyandlink=0, manipindex=16)
    #     keyframe_space_e1 = planning_G.nodes[e1]['keyframe_space']
        
    #     # generate a starting point, and a steering point, according to constraints (if applicable). 
    #     # check if the starting point has generated already:
    #     if  planning_G.nodes[e1].get('point', None) is None:
    #         print("Constraints {}}: {}".format(e1, planning_G.nodes[e1].get("constraint_ids", [])))
    #         foliation_model =  planning_G.nodes[e1].get("foliation_model", None)
    #         foliation_value =  planning_G.nodes[e1].get("foliation_value", None)

    #         with DisabledCollisionsContext(sim, [], [], disable_visualization=True):
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