from obstacle.obstacle import obstacle
from trajectory_util import *
import matplotlib as mpl
import numpy as np

class polygon_obstacle(obstacle):
    def __init__(self, corners):
        self.corners = corners

    # Ensures we never get to close to the obstacle
    def add_obstacle_constraint(self, solver, coords, buffer):
        for po in self.corners:
            for pr0, pr1 in coords:
                super().add_line_point_constraint(po[0],po[1], pr0, pr1, buffer)