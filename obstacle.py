import pylab as plt
import util
import casadi as ca
import numpy as np

class obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    # Ensures we never get to close to the obstacle
    def add_obstacle_constraint(self, opti, xpos, ypos, theta, drive, safety_distance):
        coords = util.transform_geometry([xpos, ypos, theta], drive)

        for p0, p1 in coords:
            x0 = p0[0]
            y0 = p0[1]
            x1 = p1[0]
            y1 = p1[1]
            px = x1 - x0
            py = y1 - y0
            norm = px * px + py * py
            u = ((self.x - x0) * px + (self.y - y0) * py) / norm
            u = ca.fmax(ca.fmin(u, 1), 0)
            x = x0 + u * px
            y = y0 + u * py
            dx = x - self.x
            dy = y - self.y
            dist = np.sqrt((dx * dx + dy * dy))
            opti.subject_to(dist > self.radius + safety_distance)

    # Draws the obstacle
    def draw(self, ax):
        ax.add_artist(plt.Circle((self.x, self.y), self.radius, color="r"))