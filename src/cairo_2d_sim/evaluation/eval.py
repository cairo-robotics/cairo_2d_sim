from fastdtw import fastdtw
from scipy.spatial.distance import euclidean

def path_length(trajectory):
    total_path_length = 0
    for i in range(len(trajectory) - 1):
        total_path_length += euclidean(trajectory[i], trajectory[i + 1])
        
        
def adherance_to_style(trajectory, gold_trajectory):
    dist, _ = fastdtw(trajectory, gold_trajectory, dist="euclidean")
    return dist

def success(trajectory, goal_point, error = .001):
    if euclidean(trajectory[-1] == goal_point):
        return True
    
def adherance_to_function(trajectory, constraint_set):
    pass

def omega_set_distance(point, constraint):
    pass