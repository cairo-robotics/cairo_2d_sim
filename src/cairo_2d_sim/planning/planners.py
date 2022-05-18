
import random

import numpy as np
import igraph as ig

from cairo_2d_sim.planning.constraints import project_config, val2str, name2idx


__all__ = ['CBiRRT2']



class CRRT():


    def __init__(self, state_space, state_validity_checker, interpolation_fn, distance_fn, params, logger=None):
        self.tree = ig.Graph(directed=True)
        self.state_space = state_space
        self.svc = state_validity_checker
        self.interp_fn = interpolation_fn
        self.distance_fn = distance_fn
        self.smooth_path = params.get('smooth_path', False)
        self.epsilon = params.get('epsilon', 10)
        self.extension_distance = 50
        self.smoothing_time = params.get('smoothing_time', 10)
        self.iters = 100000
    
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
        v1 = q_proj[0] - q_near[0]
        v2 = q_proj[1] - q_near[1]
        v_norm = (v1**2 + v2**2)**.5
        if v_norm == 0:
            return None
        return [q_near[0] + self.extension_distance * v1/v_norm, q_near[1] + self.extension_distance * v2/v_norm, q_proj[2]]
        
            
    def _constrain_config(self, q_candidate, tsr):
        # these functions can be very problem specific. For now we'll just assume the most very basic form.
        # futre implementations might favor injecting the constrain_config function 
        return project_config(tsr, q_candidate)
    

    def _extract_graph_path(self, tree=None, from_idx=None, to_idx=None):
        if tree is None:
            tree = self.tree
        if from_idx is None or to_idx is None:
            from_idx = name2idx(tree, self.start_name)
            to_idx = name2idx(tree, self.goal_name)
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