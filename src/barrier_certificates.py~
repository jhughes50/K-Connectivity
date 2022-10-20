import numpy as np

class BarrierCertificates:

    def __init__(self):

        self.radius_safety = 1
        self.radius_connectivity = 50


    def safety_check(self, x, y):
        return np.linalg.norm(x - y) > self.radius_safety

    def connection_check(self, x, y):
        return np.linalg.norm(x - y) <= self.radius_connectivity
