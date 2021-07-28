from casadi import *
from matplotlib.pyplot import xcorr
from drive.drive import drive
import json
from numpy import *


class skid_steer_drive(drive):
    def __init__(self, solver, length, width, wheelbase, kv, ka):
        self.solver = solver
        self.length = length
        self.width = width
        self.wheelbase = wheelbase
        self.kv = kv
        self.ka = ka

    def initialize_drive(self, N):
        self.N = N
        self.X = self.solver.variable(5,self.N+1)
        self.x = self.X[0,:]
        self.y = self.X[1,:]
        self.theta = self.X[2,:]
        self.vl = self.X[3,:]
        self.vr = self.X[4,:]
        self.U = self.solver.variable(2,self.N)
        self.al = self.U[0,:]
        self.ar = self.U[1,:]
        drive.__init__(self, self.solver, self.x, self.y, self.theta, self.length, self.width)
    
    def add_dynamics_constraint(self, dt):
        self.dt = dt
        xdot = lambda x,u: vertcat(
            cos(x[2]) * (x[3]+x[4])/2,
            sin(x[2]) * (x[3]+x[4])/2,
            (x[4]-x[3])/self.wheelbase,
            u[0],
            u[1]) # dx/dt = f(x,u)
        for k in range(self.N):
            self.solver.subject_to(self.X[:,k+1] == dt * xdot(self.X[:,k], self.U[:,k])+self.X[:,k])

    def add_voltage_constraint(self, max_abs_voltage):
        for k in range(self.N):
            self.solver.subject_to(self.solver.bounded(-max_abs_voltage, self.vl[k] * self.kv + self.al[k] * self.ka, max_abs_voltage))
            self.solver.subject_to(self.solver.bounded(-max_abs_voltage, self.vr[k] * self.kv + self.ar[k] * self.ka, max_abs_voltage))
        
    def add_regional_constraint(self, solver, x0, y0, x1, y1, max_velocity, constrainted_velocity):
        xu = (x0-x1)**2/4
        yu = (y0-y1)**2/4
        xv = (x0+x1)/2
        yv = (y0+y1)/2
        u = sign(-(self.x-xv)**2+xu)
        v = sign(-(self.y-yv)**2+yu)
        f = u * (u + 1) * v * (v + 1)
        diff = (max_velocity - constrainted_velocity) * f / 4 + constrainted_velocity
        solver.subject_to((self.vl+self.vr)/2<=diff)
        
    def set_initial_guess(self, x, y, theta):
        self.solver.set_initial(self.x, x)
        self.solver.set_initial(self.y, y)
        self.solver.set_initial(self.theta, theta)
        self.solver.set_initial(self.vl, 0)
        self.solver.set_initial(self.vr, 0)
        self.solver.set_initial(self.al, 0)
        self.solver.set_initial(self.ar, 0)

    def export_trajectory(self, sol, name):
        # data = []
        # x = sol.value(self.x)
        # y = sol.value(self.y)
        # vl = sol.value(self.vl)
        # vr = sol.value(self.vr)
        # al = sol.value(self.al)
        # ar = sol.value(self.ar)
        # theta = sol.value(self.theta)
        # for k in range(self.N+1):
        #     data.append({
        #         't':round(k*sol.value(self.dt),4),
        #         'x':round(x[k],4),
        #         'y':round(y[k],4),
        #         'theta':round(theta[k],4),
        #         'Vl': 0.0 if k>=self.N else round(vl[k] * self.kv + al[k] * self.ka,4),
        #         'Vr': 0.0 if k>=self.N else round(vr[k] * self.kv + ar[k] * self.ka,4),
        #         'vl':round(vl[k],4),
        #         'vr':round(vr[k],4),
        #         'al':0.0 if k>=self.N else round(al[k],4),
        #         'ar':0.0 if k>=self.N else round(ar[k],4),
        #     })
        # with open('Trajectory.json', 'w') as outfile:
        #     json.dump(data, outfile, indent=4)
        vl = []
        vr = []
        distance = []
        theta = []
        time = -0.02
        dt = sol.value(self.dt)
        d = [0]
        for i in range(self.N + 1):
            d.append(round(dt*(sol.value(self.vl)[i]+sol.value(self.vr)[i])/2+d[-1],4))
        while time != (dt * self.N):
            time = min(dt * self.N, time + 0.02)
            index = min(int(floor(time / dt)),249)
            percent = (time % dt) / dt
            # print(index)
            vl.append(round((sol.value(self.vl)[index+1]-sol.value(self.vl)[index])*percent+sol.value(self.vl)[index],4))
            vr.append(round((sol.value(self.vr)[index+1]-sol.value(self.vr)[index])*percent+sol.value(self.vr)[index],4))
            distance.append(round((d[index+1]-d[index])*percent+d[index],4))
            theta.append(round((sol.value(self.theta)[index+1]-sol.value(self.theta)[index])*percent+sol.value(self.theta)[index],4))

        # for r in sol.value(self.vl):
        #     print(round(r,4))
            
        f = open(name + ".java", "w")
        f.write("package frc.paths;\n")
        f.write("\n")
        f.write("public class " + name + " extends Path {\n")
        f.write("   private final static double[][] points = {\n")
        for j in range(len(vl)):
            f.write("       {" + str(vl[j]) + "," + str(vr[j]) + "," + str(distance[j]) + "," + str(theta[j]) + "},\n")
        f.write("   };\n")
        f.write("   public double[][] getPath() {\n")
        f.write("       return points;\n")
        f.write("   }\n")
        f.write("}\n")
        f.close()