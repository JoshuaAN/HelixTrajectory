from casadi import * 
from drive.drive import drive

class swerve_drive(drive):
    def __init__(self, solver):
        self.solver = solver

    def initialize_drive(self, N):
        self.N = N
        self.X = self.solver.variable(11,N+1)
        self.x = self.X[0,:]
        self.y = self.X[1,:]
        self.theta = self.X[2,:]
        self.vl = self.X[3,:]
        self.vr = self.X[4,:]
        self.U = self.solver.variable(8,N)
        self.al = self.U[0,:]
        self.ar = self.U[1,:]
        drive.__init__(self, self.solver, self.x, self.y, self.theta, self.length, self.width)

    def add_dynamics_constraint(self, dt):
        f = lambda x, u: vertcat(

        )

    def set_initial_guess(self, x, y, theta):
        