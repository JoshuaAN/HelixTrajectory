from obstacle import obstacle
from casadi import *
import matplotlib as mpl
import pylab as plt

from differential_drive import differential_drive
import util

N = 500 # number of control intervals

opti = Opti()

drive = differential_drive(35.0/12.0, 33.0/12.0, 28.0/12.0)

initial_pose = [3, 7.5, 0] # xpos, ypos, theta
final_pose = [3, 7.5, 3.14] # xpos, ypos, theta

middle_points = [3,5]

obstacles_coords = [[12.5,5],[20,10],[25,5]]

X = opti.variable(5,N+1)
xpos   = X[0,:]
ypos = X[1,:]
theta = X[2,:]
vl = X[3,:]
vr = X[4,:]
U = opti.variable(2,N)
al = U[0,:]
ar = U[1,:]
T = opti.variable()

opti.minimize(T) # minimizing time is the trajectory planner's objective

# ---- dynamic constraints --------
f = lambda x,u: vertcat(cos(x[2]) * (x[3]+x[4])/2,sin(x[2]) * (x[3]+x[4])/drive.wheelbase,(x[4]-x[3])/2,u[0],u[1]) # dx/dt = f(x,u)

dt = T/N # length of a control interval
for k in range(N): # loop over control intervals
   x_next = X[:,k] + f(X[:,k],U[:,k]) * dt
   opti.subject_to(X[:,k+1]==x_next) # close the gaps

obstacles = []
for x, y in obstacles_coords:
   obstacles.append(obstacle(x, y, 1.5/12.0))
   obstacles[-1].add_obstacle_constraint(opti, xpos, ypos, theta, drive, 0.3)

opti.subject_to(opti.bounded(-10,al,10))
opti.subject_to(opti.bounded(-10,ar,10))
opti.subject_to(opti.bounded(-10,vl,10))
opti.subject_to(opti.bounded(-10,vr,10))

opti.subject_to(xpos[0]==initial_pose[0])
opti.subject_to(ypos[0]==initial_pose[1])
opti.subject_to(theta[0]==initial_pose[2])
opti.subject_to(xpos[-1]==final_pose[0])
opti.subject_to(ypos[-1]==final_pose[1])
opti.subject_to(theta[-1]==final_pose[2])
opti.subject_to(vl[0]==0)
opti.subject_to(vl[-1]==0)
opti.subject_to(vr[0]==0)
opti.subject_to(vr[-1]==0)

opti.subject_to(T>=0) # Time must be positive

x_init, y_init, theta_init = util.load_init_json(
   "init.json", initial_pose[2], final_pose[2], N+1
)

opti.set_initial(xpos, x_init)
opti.set_initial(ypos, y_init)
opti.set_initial(theta, theta_init)

opti.set_initial(vl, 0)
opti.set_initial(vr, 0)
opti.set_initial(al, 0)
opti.set_initial(ar, 0)
opti.set_initial(T, 10)

opti.solver("ipopt")
sol = opti.solve()
print(sol.value(T))
util.draw_trajectory(sol.value(xpos),sol.value(ypos),sol.value(theta),obstacles,drive,"trajectory")
# util.draw_trajectory.
# util.animate_trajectory(sol.value(xpos),sol.value(ypos),sol.value(theta),obstacles,drive,sol.value(T)/(N+1))
# util.animate_trajectory(x_init,y_init,theta_init,obstacles,drive,10/(N+1), "trajectory")
# plt.plot((sol.value(vl)+sol.value(vr))/2)

plt.show()