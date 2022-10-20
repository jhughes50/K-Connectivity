import numpy as np
from agent import Agent


class Swarm(Agent):

    def __init__(self):

        super().__init__()

        self.positions = []
