from casadi import *

d = 3

tau = collocation_points(d, 'legendre')

[C,B,D] = collocation_coeff(tau)

T = 10

x1 = SX.sym('x1')
x2 = SX.sym('x2')
x = vertcat(x1, x2)
u = SX.sym('u')

xdot = vertcat((1-x2**2)*x1 - x2 + u, x1)

L = x1**2 + x2**2 + u**2

f = Function('f', [x, u], [xdot, L], ['x', 'u'], ['xdot', 'L'])

N = 20
h = T/N

opti = Opti()
J = 0

Xk = opti.variable(2)
opti.subject_to(Xk==[0, 1])
opti.set_initial(Xk, [0, 1])

Xs = {Xk}
Us = {}

Uk = opti.variable()
opti.subject_to(opti.bounded(-1,Uk,1))
opti.set_initial(Uk, 0)

for k in range(N):
   # New NLP variable for the control
   
   Us[-1] = Uk

   # Decision variables for helper states at each collocation point
   Xc = opti.variable(2, d)
   opti.subject_to(-0.25 <= Xc[1,:])
   opti.set_initial(Xc, repmat([0,0],1,d))

   # Evaluate ODE right-hand-side at all helper states
   [ode, quad] = f(Xc, Uk)

   # Add contribution to quadrature function
   J = J + quad*B*h

   # Get interpolating points of collocation polynomial
   Z = vertcat(Xk, Xc)

   # Get slope of interpolating polynomial (normalized)
   Pidot = Z*C
   # Match with ODE right-hand-side 
   opti.subject_to(Pidot == h*ode)

   # State at end of collocation interval
   Xk_end = Z*D

   # New decision variable for state at end of interval
   Xk = opti.variable(2)
   Xs[-1] = Xk
   opti.subject_to(-0.25 <= Xk(1))
   opti.set_initial(Xk, [0,0])

   # Continuity constraints
   opti.subject_to(Xk_end==Xk)

opti.minimize(J)

opti.solver('ipopt')

sol = opti.solve()