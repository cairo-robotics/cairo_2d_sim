
import random

import numpy as np
import igraph as ig

from cairo_2d_sim.planning.constraints import project_config, val2str, name2idx


__all__ = ['CBiRRT2']


def parametric_xytheta_lerp(q0, q1, steps):
    """
    This function directly interpolates between the start q0 and q1, element-wise parametrically
    via the discretized interval determined by the number of steps.

    Args:
        q0 (ndarray): Numpy vector representing the starting point.
        q1 (ndarray): Numpy vector representing the ending point.
        steps (int): Number of discrete steps to take.

    Returns:
        [ndarray]: Numpy array of the interpolation between q0 and q1.
    """
    times = [x / (steps - 1)
             for x in range(0, steps)]  # % normalized time from 0 -> 1
    C = q0[2]
    T = q1[2]
    d = ((T - C + 540) % 360) - 180
    if d > 0:
        theta_interp = np.array([np.array([abs(C + d * t) % 360]) for t in times])
    if d < 0:
        theta_interp = np.array([np.array([abs(C - d * t) % 360]) for t in times])
    xy_interp = np.array([t*(q1[:2]-q0[:2]) + q0[:2] for t in times])
    np.concatenate((xy_interp, theta_interp), axis=1)
    return 

def xytheta_distance(q0, q1):
    """
    

    Args:
        q0 (ndarray): Numpy vector representing the starting point.
        q1 (ndarray): Numpy vector representing the ending point.

    Returns:
        [ndarray]: Numpy array of the interpolation between q0 and q1.
    """
    euclid_distance = abs(np.linalg.norm(np.array(q0[:2]) - np.array(q1[0:2])))
    theta_diff = q0[2] - q1[2]
    theta_delta = abs((theta_diff + 180) % 360 - 180)
    return euclid_distance + theta_delta

class CRRT():


    def __init__(self, state_space, state_validity_checker, interpolation_fn, distance_fn, params, logger=None):
        self.tree = ig.Graph(directed=True)
        self.state_space = state_space
        self.svc = state_validity_checker
        self.interp_fn = interpolation_fn
        self.distance_fn = distance_fn
        self.smooth_path = params.get('smooth_path', False)
        self.epsilon = params.get('epsilon', 250)
        self.smoothing_time = params.get('smoothing_time', 10)
        self.iters = 5000
    
    def plan(self, tsr, start_q, goal_q):
        """ 
        Args:
            robot (Cairo Planning Manipulator): Need to for embedded IK/FK functionality.
    
        """
        self.start_q = start_q
        self.goal_q = goal_q
        self._initialize_tree(start_q, goal_q)
        self.tree = self.crrt(tsr)
        print("Size of tree: {}".format(len(self.tree.vs)))
        if self.tree is not None:
            graph_path = self._extract_graph_path()
            if len(graph_path) == 1:
                return None
            else:
                if self.smooth_path:
                    self._smooth_path(graph_path, tsr, self.smoothing_time)
                #print("Graph path found: {}".format(graph_path))
                return self._extract_graph_path()
        # plan = self.get_plan(graph_path)
        # #self._smooth(path)
        # return plan

    def crrt(self, tsr):
        iters=0
        continue_to_plan = True
        while continue_to_plan:
            iters += 1
            if iters > self.iters:
                raise Exception("Max iterations reached for CRRT")
            q_rand = self._random_config()
            q_near = self._neighbors(self.tree, q_rand)
            q_proj = self._constrained_extend(tsr, q_near, q_rand)
            if q_proj is not None:
                self._add_vertex(self.tree, q_proj)
                # print(q_near, q_proj)
                # print(self._distance(q_near, q_proj))
                print(self._distance(q_proj, self.goal_q))
                self._add_edge(self.tree, q_near, q_proj, self._distance(q_near, q_proj))
            if q_proj is not None and self._equal(q_proj, self.goal_q):
                self._add_vertex(self.tree, self.goal_q)
                self._add_edge(self.tree, q_proj, self.goal_q, self._distance(q_proj, self.goal_q))
                return self.tree
            
    def reset_planner(self):
        self.start_q = None
        self.goal_q = None
        self.tree = ig.Graph(directed=True)
    
    def _constrained_extend(self, tsr, q_near, q_target):
        q_proj = self._constrain_config(q_candidate=q_target, tsr=tsr)
        if self._distance(q_proj, self.goal_q) < 2 * self._distance(q_near, self.goal_q):
            return q_proj
        else:
            return None
            
    def _constrain_config(self, q_candidate, tsr):
        # these functions can be very problem specific. For now we'll just assume the most very basic form.
        # futre implementations might favor injecting the constrain_config function 
        return project_config(tsr, q_candidate)
        

    def _extract_graph_path(self, tree=None, from_idx=None, to_idx=None):
        if tree is None:
            tree = self.tree
        if from_idx is None or to_idx is None:
            from_idx = name2idx(tree, self.start_name)
            print(from_idx)
            to_idx = name2idx(tree, self.goal_name)
            print(to_idx)
        if 'weight' in tree.es.attributes():
            return tree.get_shortest_paths(from_idx, to_idx, weights='weight', mode='ALL')[0]
        else:
            return tree.get_shortest_paths(from_idx, to_idx, mode='ALL')[0]

    def get_path(self, plan):
        points = [self.tree.vs[idx]['value'] for idx in plan]
        pairs = list(zip(points, points[1:]))
        segments = [self.interp_fn(np.array(p[0]), np.array(p[1]))
                    for p in pairs]
        segments = [[list(val) for val in seg] for seg in segments]
        path = []
        for seg in segments:
            path = path + seg
        return path

    def _neighbors(self, tree, q_s, fraction_random=.1):
        if len(tree.vs) == 1:
            return [v for v in tree.vs][0]['value']
        if random.random() <= fraction_random:
            return random.choice([v for v in tree.vs])['value']
        return sorted([v for v in tree.vs], key= lambda vertex: self._distance(vertex['value'], q_s))[0]['value']

    def _random_config(self):
        # generate a random config to extend towards. This can be biased by whichever StateSpace and/or sampler we'd like to use.
        return np.array(self.state_space.sample())

    def _initialize_tree(self, start_q, goal_q):
        self.start_name = val2str(start_q)
        self.goal_name = val2str(goal_q)
        self._add_vertex(self.tree, start_q)
   

    def _equal(self, q1, q2):
        if self._distance(q1, q2) <= self.epsilon:
            return True
        return False

    def _validate(self, sample):
        return self.svc.validate(sample)
    
    def _add_vertex(self, tree, q):
        tree.add_vertex(val2str(q), **{'value': q})


    def _add_edge(self, tree, q_from, q_to, weight):
        q_from_idx = name2idx(tree, val2str(q_from))
        q_to_idx = name2idx(tree, val2str(q_to))
        if val2str(q_from) == self.start_name and val2str(q_to) == self.goal_name:
            tree.add_edge(q_from_idx, q_to_idx, **{'weight': weight})
        elif tuple(sorted([q_from_idx, q_to_idx])) not in set([tuple(sorted(edge.tuple)) for edge in tree.es]):
            tree.add_edge(q_from_idx, q_to_idx, **{'weight': weight})
    
    def _distance(self, q1, q2):
        return self.distance_fn(q1, q2)