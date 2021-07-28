from math import pi, remainder
import casadi as ca
from trajectory_util import solve_corners

class drive:
    def __init__(self, solver, x, y, theta, length, width):
        self.solver = solver
        self.x = x
        self.y = y
        self.theta = theta
        self.length = length
        self.width = width

    def add_obstacle_constraint(self, obstacle, buffer):
        coords = solve_corners(self.x, self.y, self.theta, self.length, self.width)
        obstacle.add_obstacle_constraint(self.solver, coords, buffer)

    def add_boundry_constraint(self, initial_pose, final_pose):
        self.solver.subject_to(self.x[0] == initial_pose[0])
        self.solver.subject_to(self.y[0] == initial_pose[1])
        self.solver.subject_to(self.theta[0] == initial_pose[2])
        self.solver.subject_to(self.x[-1] == final_pose[0])
        self.solver.subject_to(self.y[-1] == final_pose[1])
        self.solver.subject_to(ca.mod(self.theta[-1], 2 * pi) == final_pose[2])