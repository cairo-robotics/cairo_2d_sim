import time

import numpy as np
from sklearn.neighbors import NearestNeighbors as NN
from sklearn.neighbors import KDTree
from pynndescent import NNDescent

# class NearestNeighbors():
#     """
#     Wrapper class to interface into various Nearest Neighbor models.
    
#     TODO: Currently only supports KDTree. This may be all that is needed for now.

#     Args:
#         X (array-like): NxD array-like data. N is number of samples, D is number of dimensions
#         model_type (str, optional): [description]. Determines the choice of model. Defaults to "KDTree".
#         model_args (list, optional): [description]. Args to pass to the chosen model. Defaults to None.
#         model_kwargs (dict, optional): [description]. Keyword args to pass to the chosen model. Defaults to None.
#     """

#     def __init__(self, X, model_type="KDTree", model_args=None, model_kwargs=None):
#         """
#         Will fit initial model upon instantiation. 
    
#         Args:
#             X (array-like): NxD array-like data. N is number of samples, D is number of dimensions
#             model_type (str, optional): [description]. Determines the choice of model. Defaults to "KDTree".
#             model_args (list, optional): [description]. Args to pass to the chosen model. Defaults to None.
#             model_kwargs (dict, optional): [description]. Keyword args to pass to the chosen model. Defaults to None.
        
#         Raises:
#             ValueError: Error if model type not available for use 
#         """
#         self.available_models = ['KDTree']
#         self.X = X
#         if model_type not in self.available_models:
#             raise ValueError(
#                 "{} is not a valid value for model_type. Must be one of {}".format(model_type, self.available_models))
#         else:
#             self.model_type = model_type
#         self.model_args = model_args if model_args is not None else []
#         self.model_kwargs = model_kwargs if model_kwargs is not None else {}
#         self.fit()

#     def fit(self):
#         """
#         Fits the chosen model on the current data set X.
#         """
#         if self.model_type == "KDTree":
#             self.model = KDTree(self.X, *self.model_args, **self.model_kwargs)


#     def append(self, x):
#         """
#         Adds x to the dataset X. Will NOT refit the model unless explicitly asked to do so via fit().
    
#         Args:
#             x (array-like): 1xD vector.
#         """
#         self.X = np.concatenate((self.X, [x]), axis=0)

#     def query(self, x_test, k=3):
#         """[
#         Queries the fitted nearest neighbor model for k-nearest neighbors. 

#         Args:
#             x_test (array-like): 1xD vector test query.
#             k (int): The number of neighbors.

#         Returns:
#             [ndarray], [ndarray]: Returns the ndarray of distances to each neighbor and ndarray of neighbor points.
#         """
#         distances, indices = self.model.query([x_test], k=k)
#         return  distances[0], [list(self.X[idx]) for idx in indices[0]]

# class NearestNeighborsDescent():
#     """
#     Wrapper class to interface into various Nearest Neighbor Decent models.
    

#     Args:
#         X (array-like): NxD array-like data. N is number of samples, D is number of dimensions
#     """

#     def __init__(self, X):
#         """
#         Will fit initial model upon instantiation. 
    
#         Args:
#       .
        
#         Raises:
#             ValueError: Error if model type not available for use 
#         """
#         self.X = X
#         self.fit()

#     def fit(self):
#         """
#         Fits the chosen model on the current data set X.
#         """
#         start = time.process_time()
#         self.index = NNDescent(self.X, n_jobs=-1, n_neighbors=100) 
#         print("ANN Decent creation time: {}".format(time.process_time() - start))

#     def append(self, x):
#         """
#         Adds x to the dataset X. Will NOT refit the model unless explicitly asked to do so via fit().
    
#         Args:
#             x (array-like): 1xD vector.
#         """
#         self.X = np.concatenate((self.X, [x]), axis=0)

    # def query(self, x_test, k=3):
    #     """[
    #     Queries the fitted nearest neighbor model for k-nearest neighbors. 

    #     Args:
    #         x_test (array-like): 1xD vector test query.
    #         k (int): The number of neighbors.

    #     Returns:
    #         [ndarray], [ndarray]: Returns the ndarray of distances to each neighbor and ndarray of neighbor points.
    #     """
    #     neighbors = self.index.query(x_test, k=k, diversify_prob=1.0,
    # pruning_degree_multiplier=0.5)

    #     return neighbors
    
class NearestNeighbors():

    
    def __init__(self, X):
        self.X = X
        self.model = NN(n_neighbors=100, algorithm='ball_tree')
        self.fit()
    
    def fit(self):
        start = time.process_time()
        self.model.fit(self.X)
    
    def query(self, x_test, k=3):
        nng = self.model.kneighbors(np.array([x_test]))
        distances = nng[0][0][1:]
        neighbor_indices = nng[1][0][1:]
        neighbors = [list(self.X[idx]) for idx in neighbor_indices]
        return  distances[:k+1], neighbors[:k+1]

    def get_graph(self):
        return self.model.kneighbors(self.X)
