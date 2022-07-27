from math import dist
import os
import json
import time

from fastdtw import fastdtw
from scipy.spatial.distance import euclidean


class IPDRelaxEvaluation():
    
    def __init__(self, output_dir):
        self.output_dir = output_dir
        self.trials = []
        
    def add_trial(self, trial):
        self.trials.append(trial)

    def export(self):
        # TODO: MAKE EACH TRIAL OUTPUT
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

    
    
class IPDRelaxEvaluationTrial():
    
    def __init__(self):
        self.path_length = -1
        self.a2s_distance = -1
        self.success = False
        self.a2f_percentage = -1
        self.planning_time = -1
        self.ip_gen_times = []
        self.trajectory = []
        self.notes = "None"
        self.timers = {}
        
    def eval_path_length(self, trajectory):
        total_path_length = 0
        for i in range(len(trajectory) - 1):
            total_path_length += euclidean(trajectory[i], trajectory[i + 1])
        return total_path_length

    def add_path_length(self, total_path_length):
        self.path_length = total_path_length

    def eval_a2s(self, trajectory, gold_trajectory):
        dist, _ = fastdtw(trajectory, gold_trajectory)
        return dist
    
    def add_a2s(self, dist):
        self.a2s_distance = dist

    def add_success(self, success_bool):
        self.success = success_bool
    
    def eval_success(self, trajectory, goal_point, epsilon=10):
        dist_xy = euclidean(trajectory[-1][:2], goal_point[:2])
        delta_theta = abs(trajectory[-1][2] - goal_point[2])
        diff_theta = abs((delta_theta + 180) % 360 - 180)
        if dist_xy < epsilon and diff_theta < 10:
            return True
        else:
            return False
    
    def eval_a2f(self, trajectory_segments, constraint_eval_map, constraint_ordering):
        results = []
        for idx, segment in enumerate(trajectory_segments):
            if constraint_ordering[idx] is not None:
                evaluator = constraint_eval_map.get(constraint_ordering[idx], None)
                if evaluator is not None:
                    for point in segment:
                        results.append(evaluator.validate(point))
            else:
                for point in segment:
                    results.append(True)
        return sum(results) / len(results)
    
    def add_a2f(self, percentage):
        self.a2f_percentage = percentage
                
    def add_planning_time(self, time):
        self.planning_time = time
        
    def add_steering_point_gen_times(self, times):
        self.ip_gen_times.append(times)
        
    def start_timer(self, name):
        self.timers[name] = time.perf_counter()
    
    def end_timer(self, name):
        tic = self.timers[name]
        toc = time.perf_counter()
        return toc - tic
    
