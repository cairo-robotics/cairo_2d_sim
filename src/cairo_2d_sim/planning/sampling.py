import random

import numpy as np

class DistributionSampler():

    def __init__(self, distribution_model, fraction_uniform=.45, high_confidence_sampling=False):
        """
        Samples from a fitted model that represents the distribution of discrete points. This could be a keyframe distribution, trajectory distribution, or any arbitrary distrubtion. Sampling checks if the values are within limits (usually the joint limits of the robot) passed in as an argument ot the sample function.

        Args:
            distribution_model (object): The distribution model. Expects a sample() member function.
        """
        self.model = distribution_model
        self.fraction_uniform = fraction_uniform
        self.high_confidence_sampling = high_confidence_sampling
    
    def sample(self, dimension_limits):
        if self.high_confidence_sampling:
            samples = []
            for _ in range(0, 250):
                samples.append(self.model.sample())

            np_samples = np.array(samples)

            scores = self.model.score_samples(np_samples)
            order = np.argsort(-scores)
            samples = np_samples[order]
            rank_sorted_sampled = np.asarray(samples)
            return list(rank_sorted_sampled[0])
        else:
            count = 1
            within_limits = False
            while not within_limits:
                count += 1
                if random.random() > self.fraction_uniform:
                    sample = self.model.sample()
                else:
                    sample = self._uniform_random_q(dimension_limits)
                within_limits = self._within_limits(sample, dimension_limits)
                if within_limits:
                    return sample
                if count >= 10000:
                    raise RuntimeError(
                        "Could not effectively sample a single point within the joint limits after 10000 attempts.")
        
    
 
    def _within_limits(self, sample, limits):
        for idx, limit in enumerate(limits):
            if sample[idx] < limit[0] or sample[idx] > limit[1]:
                return False
        return True
    
    def _uniform_random_q(self, dimension_limits):
        return np.array([random.uniform(limit[0], limit[1]) for limit in dimension_limits])