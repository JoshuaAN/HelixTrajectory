from obstacle.obstacle import obstacle
import pylab as plt

class circular_obstacle(obstacle):
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    # Ensures we never get to close to the obstacle
    def add_obstacle_constraint(self, solver, coords, buffer):
        for p0, p1 in coords:
            super().add_line_point_constraint(solver, self.x, self.y, p0, p1, self.radius + buffer)

    # Draws the obstacle
    def draw(self, ax):
        ax.add_artist(plt.Circle((self.x, self.y), self.radius, color="r"))