from casadi import *
import pylab as plt

import util
from trajectory_io import *

N = 250 # number of control intervals

opti = Opti()

drive = import_robot(opti, "Robot.json")
# obstacles = import_obstacles("Field.json")
obstacles = []

initial_pose = [3, 7.5, 0] # xpos, ypos, theta
final_pose = [3, 7.5, -3.14] # xpos, ypos, theta

# obstacles_coords = [[20,10],[12.5,5],[25,5]]
obstacles_coords = [[8.25,7.5]]

T = opti.variable()
dt = T/N

opti.minimize(T) # minimizing time is the trajectory planner's objective

drive.initialize_drive(N)
drive.add_dynamics_constraint(dt)
drive.add_boundry_constraint(initial_pose, final_pose)
# drive.add_voltage_constraint(10)
for x, y in obstacles_coords:
   obstacles.append(circular_obstacle(x, y, 0.125))
for obstacle in obstacles:
   drive.add_obstacle_constraint(obstacle, 1)

opti.subject_to(opti.bounded(-5,drive.vl,5))
opti.subject_to(opti.bounded(-5,drive.vr,5))
opti.subject_to(opti.bounded(-5,drive.al,5))
opti.subject_to(opti.bounded(-5,drive.ar,5))

opti.subject_to(drive.vl[0]==0)
opti.subject_to(drive.vr[0]==0)
opti.subject_to(drive.vl[-1]==0)
opti.subject_to(drive.vr[-1]==0)

opti.subject_to(T>=0) # Time must be positive

x_init, y_init, theta_init = util.load_init_json(
   "init.json", initial_pose[2], final_pose[2], N+1
)

drive.set_initial_guess(x_init,y_init,theta_init)

opti.set_initial(T, 10)

opti.solver("ipopt")
sol = opti.solve()
drive.export_trajectory(sol, "Traj")
print(sol.value(T))
print(sol.value(dt) * N)
# util.draw_trajectory(sol.value(drive.x),sol.value(drive.y),sol.value(drive.theta),obstacles,drive,"trajectory")
util.animate_trajectory(sol.value(drive.x),sol.value(drive.y),sol.value(drive.theta),obstacles,drive,sol.value(T)/(N+1), "trajectory")
# util.animate_trajectory(x_init,y_init,theta_init,obstacles,drive,sol.value(T)/(N+1), "trajectory")

plt.show()