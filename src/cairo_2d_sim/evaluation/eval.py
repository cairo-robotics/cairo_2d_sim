from math import dist
import os
import json
import time

from fastdtw import fastdtw
from scipy.spatial.distance import euclidean


class IPDRelaxEvaluation():
    
    def __init__(self, output_dir):
        self.output_dir = output_dir
        self.path_lengths = []
        self.a2s_dists = []
        self.successes = []
        self.a2f_percentages = []
        self.planning_times = []
        self.ip_gen_times = []
        self.timers = {}
        
    def add_path_length(self, trajectory):
        total_path_length = 0
        for i in range(len(trajectory) - 1):
            total_path_length += euclidean(trajectory[i], trajectory[i + 1])
        self.path_lengths.append(total_path_length)
        
    def add_a2s(self, trajectory, gold_trajectory):
        dist, _ = fastdtw(trajectory, gold_trajectory)
        self.a2s_dists.append(dist)

    def add_success(self, trajectory, goal_point, epislon=.5):
        dist_xy = euclidean(trajectory[-1][:2], goal_point[:2])
        delta_theta = abs(trajectory[-1][2] - goal_point[2])
        diff_theta = abs((delta_theta + 180) % 360 - 180)
        if dist_xy < epislon and diff_theta < 10:
            self.successes.append(True)
        else:
            self.successes.append(False)
    
    def add_a2f(self, trajectory_segments, constraint_eval_map, constraint_ordering):
        results = []
        for idx, segment in enumerate(trajectory_segments):
            evaluator = constraint_eval_map[constraint_ordering[idx]]
            for point in segment:
                results.append(evaluator.evaluate(point))
        self.a2f_percentages.append(sum(results) / len(results))
                
    def add_planning_time(self, time):
        self.planning_times.append(time)
        
    def add_steering_point_gen_time(self, time):
        self.ip_gen_times.append(time)
        
    def start_timer(self, name):
        self.timers[name] = time.perf_counter()
    
    def end_timer(self, name):
        tic = self.timers[name]
        toc = time.perf_counter()
        return toc - tic
    
    def export(self):
        filename = os.path.join(self.output_dir, "eval_results.json")
        with open(filename, 'w') as f:
            data = {}
            data["path_lengths"] = self.path_lengths
            data["a2s_distances"] = self.a2s_dists
            data["successes"] = self.successes
            data["a2f_percentages"] = self.a2f_percentages
            data["planning_times"] = self.planning_times
            data["ip_generation_times"] = self.ip_gen_times
            
            json.dump(data, f)
