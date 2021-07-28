from abc import abstractmethod
from casadi import *
import numpy as np

class obstacle:
    def add_line_point_constraint(self, solver, x, y, p0, p1, distance):
            px = p1[0] - p0[0]
            py = p1[1] - p0[1]
            norm = px * px + py * py
            u = ((x - p0[0]) * px + (y - p0[1]) * py) / norm
            u = fmax(fmin(u, 1), 0)
            fx = p0[0] + u * px
            fy = p0[1] + u * py
            dx = fx - x
            dy = fy - y
            dist = np.sqrt((dx * dx + dy * dy))
            solver.subject_to(dist > distance)

    @abstractmethod
    def add_obstacle_constraint(self, solver, coords, buffer):
        pass

    @abstractmethod
    def draw(self, ax):
        pass