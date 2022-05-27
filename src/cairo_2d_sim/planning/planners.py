
import random

import numpy as np
import igraph as ig

from cairo_2d_sim.planning.constraints import project_config, val2str, name2idx
from cairo_2d_sim.planning.neighbors import NearestNeighbors
from cairo_2d_sim.planning.curve import cumulative_distance

__all__ = ['CRRT', 'CPRM']



class CRRT():


    def __init__(self, state_space, state_validity_checker, interpolation_fn, distance_fn, params, logger=None):
        self.tree = ig.Graph(directed=True)
        self.state_space = state_space
        self.svc = state_validity_checker
        self.interp_fn = interpolation_fn
        self.distance_fn = distance_fn
        self.smooth_path = params.get('smooth_path', False)
        self.epsilon = params.get('epsilon', (50, 10))
        self.extension_distance = params.get('extension_distance', 50)
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
            q_target = self._random_config()
            q_near = self._neighbors(self.tree, q_target)
            q_proj = self._constrained_extend(tsr, q_near, q_target)
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
        self.start_name = None
        self.goal_q = None
        self.goal_name = None
        self.tree = ig.Graph(directed=True)
    
    def _constrained_extend(self, tsr, q_near, q_target):
        return self._constrain_config(tsr=tsr, q_target=q_target,  q_near=q_near)
        
            
    def _constrain_config(self, tsr, q_target, q_near)              :
        # these functions can be very problem specific. For now we'll just assume the most very basic form.
        # futre implementations might favor injecting the constrain_config function 
        return project_config(tsr, q_target, q_near, self.extension_distance)
    

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
        if self.distance_fn(q1, q2) <= self.epsilon:
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
    

class CPRM():
    def __init__(self, state_space, state_validity_checker, interpolation_fn, distance_fn, params):
        self.graph = ig.Graph()
        self.state_space = state_space
        self.svc = state_validity_checker
        self.interp_fn = interpolation_fn
        self.distance_fn = distance_fn
        self.n_samples = params.get('n_samples', 4000)
        self.k = params.get('k', 5)
        self.ball_radius = params.get('ball_radius', 25)
        self.smooth = params.get('smooth_path', False)
        print("N: {}, k: {}, r: {}".format(
            self.n_samples, self.k, self.ball_radius))

    def plan(self, tsr, q_start, q_goal):
        # Initial sampling of roadmap and NN data structure.
        print("Initializing roadmap...")
        self._init_roadmap(q_start, q_goal)
        if np.linalg.norm(np.array(q_start[0:2]) - np.array(q_goal[0:2])) < 10:
            self._add_edge_to_graph(q_start, q_goal, self.distance_fn(q_start, q_goal))
            return self.best_sequence()
        print("Generating valid random samples...")
        self.samples = self._generate_samples(tsr)
        # Create NN datastructure
        print("Creating NN datastructure...")
        self.nn = NearestNeighbors(X=np.array(
            self.samples), model_kwargs={"leaf_size": 100})
        # Generate NN connectivity.
        print("Generating nearest neighbor connectivity...")
        connections = self._generate_connections(samples=self.samples)
        print("Generating graph from samples and connections...")
        self._build_graph(self.samples, connections)
        print("Attaching start and end to graph...")
        self._attach_start_and_end()
        print("Finding feasible best path in graph if available...")
        if self._success():
            if self.smooth:
                plan = self._smooth(self.best_sequence())
            else:
                plan = self.best_sequence()
            return plan
        else:
            return []

    def best_sequence(self):
        return self.graph.get_shortest_paths(self.start_name, self.goal_name, weights='weight', mode='ALL')[0]

    def get_path(self, plan):
        points = [self.graph.vs[idx]['value'] for idx in plan]
        pairs = list(zip(points, points[1:]))
        segments = [self.interp_fn(np.array(p[0]), np.array(p[1]))
                    for p in pairs]
        segments = [[list(val) for val in seg] for seg in segments]
        path = []
        for seg in segments:
            path = path + seg
        return path

    def _init_roadmap(self, q_start, q_goal):
        self.start_name = val2str(q_start)
        self.graph.add_vertex(**{'name': self.start_name, 'value': list(q_start)})
        self.goal_name = val2str(q_goal)
        self.graph.add_vertex(**{'name': self.goal_name, 'value': list(q_goal)})
       
    def _smooth(self, plan):
        def shortcut(plan):
            idx_range = [i for i in range(0, len(plan))]
            indexed_plan = list(zip(idx_range, plan))
            for curr_idx, vid1 in indexed_plan:
                for test_idx, vid2 in indexed_plan[::-1]:
                    p1 = self.graph.vs[vid1]["value"]
                    p2 = self.graph.vs[vid2]["value"]
                    valid, _ = self._extend(np.array(p1), np.array(p2))
                    if test_idx == curr_idx + 1:
                        break
                    if valid and curr_idx < test_idx:
                        del plan[curr_idx+1:test_idx]
                        return False, plan
            return True, plan

        finished = False
        current_plan = plan
        while not finished:
            finished, new_plan = shortcut(current_plan)
            if new_plan is None:
                finished = True
                break
            current_plan = new_plan
        return current_plan

    def _generate_samples(self, tsr):
        # sampling_times = [0]
        count = 0
        valid_samples = []
        while count < self.n_samples:
            # start_time = timer()
            q_rand = self._sample(tsr)
            if q_rand is not None and np.any(q_rand):
                if self._validate(q_rand):
                    if count % 500 == 0:
                        print("{} valid samples...".format(count))
                    valid_samples.append(q_rand)
                    count += 1
                    # sampling_times.append(timer() - start_time)
            # print(sum(sampling_times) / len(sampling_times))
        return valid_samples

    def _generate_connections(self, samples):
        connections = []
        for q_rand in samples:
            for q_neighbor in self._neighbors(q_rand):
                # We ignore really close together points
                # if np.linalg.norm(np.array(q_neighbor[0:2]) - np.array(q_rand[0:2])) >= 5:
                valid, local_path = self._extend(
                    np.array(q_rand), np.array(q_neighbor))
                if valid:
                    connections.append(
                        [q_neighbor, q_rand, self._weight(local_path)])
        print("{} connections out of {} samples".format(
            len(connections), len(samples)))
        return connections

    def _build_graph(self, samples, connections):
        curr_names = self.graph.vs['name'] # we have to snag the current names and values before adding vertices
        curr_values = self.graph.vs['value']
        values = curr_values + [list(sample) for sample in samples]
        names = curr_names + [val2str(sample) for sample in samples]
        self.graph.add_vertices(len(samples)) # this will append the number of required new vertices for the new samples
        self.graph.vs["value"] = values
        self.graph.vs["name"] = names
        edges = [(self._idx_of_point(c[0]), self._idx_of_point(c[1]))
                 for c in connections]
        weights = [c[2] for c in connections]
        self.graph.add_edges(edges)
        self.graph.es['weight'] = weights

    def _attach_start_and_end(self):
        start = self.graph.vs.find('name' == self.start_name)['value']
        end = self.graph.vs.find('name' == self.goal_name)['value']
        start_added = False
        end_added = False
        for q_near in self._neighbors(start, k_override=50, within_ball=False):
            if self._idx_of_point(q_near) != 0:
                successful, local_path = self._extend(start, q_near)
                if successful:
                    start_added = True
                    self._add_edge_to_graph(
                        start, q_near, self._weight(local_path))
        for q_near in self._neighbors(end, k_override=50, within_ball=False):
            if self._idx_of_point(q_near) != 1:
                successful, local_path = self._extend(q_near, end)
                if successful:
                    end_added = True
                    self._add_edge_to_graph(
                        q_near, end, self._weight(local_path))
        if not start_added or not end_added:
            raise Exception("Planning failure! Could not add either start {} and end {} successfully to graph.".format(
                {start_added}, {end_added}))

    def _success(self):
        paths = self.graph.shortest_paths_dijkstra(
            [0], [1], weights='weight', mode='ALL')
        if len(paths) > 0 and paths[0][0] != np.inf:
            return True
        return False

    def _validate(self, sample):
        return self.svc.validate(sample)

    def _extend(self, q_near, q_rand):
        local_path = self.interp_fn(np.array(q_near), np.array(q_rand))
        valid = True
        if valid:
            return True, local_path
        else:
            return False, []

    def _neighbors(self, sample, k_override=None, within_ball=True):
        if k_override is not None:
            k = k_override
        else:
            k = self.k
        distances, neighbors = self.nn.query(sample, k=k)
        if within_ball:
            return [neighbor for distance, neighbor in zip(
                distances, neighbors) if distance <= self.ball_radius and distance > 0]
        else:
            return [neighbor for distance, neighbor in sorted(
                list(zip(distances, neighbors)), key=lambda x: x[0]) if distance > 0]

    def _sample(self, tsr):
        p = self.state_space.sample()
        return np.array(tsr.project(p))

    # def _add_edge_to_graph(self, q_near, q_sample, edge_weight):
    #     q_near_idx = self._idx_of_point(q_near)
    #     q_sample_idx = self._idx_of_point(
    #         q_sample)
    #     if tuple(sorted([q_near_idx, q_sample_idx])) not in set([tuple(sorted(edge.tuple)) for edge in self.graph.es]):
    #         self.graph.add_edge(q_near_idx, q_sample_idx,
    #                             **{'weight': edge_weight})
            
    def _add_edge_to_graph(self, q_from, q_to, weight):
        if val2str(q_from) == self.start_name:
            q_from_idx = name2idx(self.graph, self.start_name)
        elif val2str(q_from) == self.goal_name:
            q_from_idx = name2idx(self.graph, self.goal_name)
        else:
            q_from_idx = name2idx(self.graph, val2str(q_from))
        if val2str(q_to) == self.start_name:
            q_to_idx = name2idx(self.graph, self.start_name)
        elif val2str(q_to) == self.goal_name:
            q_to_idx = name2idx(self.graph, self.goal_name)
        else:
            q_to_idx = name2idx(self.graph, val2str(q_to))
        if tuple(sorted([q_from_idx, q_to_idx])) not in set([tuple(sorted(edge.tuple)) for edge in self.graph.es]):
            self.graph.add_edge(q_from_idx, q_to_idx, **{'weight': weight})

    def _weight(self, local_path):
        return cumulative_distance(local_path, self.distance_fn)

    def _idx_of_point(self, point):
        return name2idx(self.graph, val2str(point))
