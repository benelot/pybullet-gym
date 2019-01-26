import numpy as np


def relu(x):
    return np.maximum(x, 0)


class SmallReactivePolicy:
    """Simple multi-layer perceptron policy, no internal state"""
    def __init__(self, observation_space, action_space, weights, biases):
        self.weights = weights
        self.biases = biases

    def act(self, ob):
        x = ob
        x = relu(np.dot(x, self.weights[0]) + self.biases[0])
        x = relu(np.dot(x, self.weights[1]) + self.biases[1])
        x = np.dot(x, self.weights[2]) + self.biases[2]
        return x
