from sklearn.neighbors import KernelDensity
from sklearn.mixture import BayesianGaussianMixture


class KernelDensityDistribution():
    """
    Wrapper around around sklearn's KernelDensity model. Will bias the process of sampling points for constrained motion planning based on demonstrated trajectories.
    
    Args:
        bandwidth (float): Parameter for KDE models that determines the spead or variance of the kernel. The smaller the bandwidth, the more closely the KDE model will fit the training data.
    """
    def __init__(self, bandwidth=.25):
        self.model = KernelDensity(kernel='gaussian', bandwidth=bandwidth)

    def fit(self, X):
        self.model.fit(X)
    
    def sample(self):
        return self.model.sample(1)[0]
    
    def score_samples(self, samples):
        return self.model.score_samples(samples)
    
class BayesianGaussianDistribution():
    """
    Wrapper around around sklearn's Guassian model.
    
    Args:
        n_components (float): Max number of components to fit to the data.
    """
    def __init__(self, n_components=2):
        self.model = BayesianGaussianMixture(n_components=n_components)

    def fit(self, X):
        self.model.fit(X)
    
    def sample(self):
        return self.model.sample(1)[0]
    
    def score_samples(self, samples):
        return self.model.score_samples(samples)

