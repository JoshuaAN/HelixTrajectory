from casadi import *
from matplotlib.pyplot import thetagrids
from numpy.lib.function_base import kaiser

class differential_drive:
    def __init__(self, 
        length,
        width,
        wheelbase,
        # linear_ks,
        # linear_ka,
        # linear_kv,
        # angular_ks,
        # angular_ka,
        # angular_kv
        ):

        self.length = length # feet
        self.width = width # feet
        self.wheelbase = wheelbase # feet
        # self.linear_ks = linear_ks
        # self.linear_ka = linear_ka
        # self.linear_kv = linear_kv
        # self.angular_ks = angular_ks
        # self.angular_ka = angular_ka
        # self.angular_kv = angular_kv
    
    def add_voltage_constraint(self, opti, al, ar, vl, vr):
        # With restrictive constraints, the optimization problem becomes too non-linear for IPOPT to handle.
        # This constraint is less restrictive and accurate than the voltage constraint, but helps with convergence.
        opti.subject_to(opti.bounded(-10,al,10))
        opti.subject_to(opti.bounded(-10,ar,10))
        # opti.subject_to(opti.bounded(self.free_speed(-12),vl,self.free_speed(12)))
        # opti.subject_to(opti.bounded(self.free_speed(-12),vr,self.free_speed(12)))
        opti.subject_to(opti.bounded(-10,vl,10))
        opti.subject_to(opti.bounded(-10,vr,10))

    # def add_voltage_constraint(self, opti, al, ar, vl, vr, max_abs_voltage):
    #     opti.subject_to()

    # def free_speed(self, voltage):
    #     if abs(voltage) <= self.ks:
    #         return 0
    #     return (voltage - self.ks * sign(voltage)) / self.kv
    
    # def acceleration(self, voltage, velocity):
    #     return (voltage - self.ks * sign(velocity) - self.kv * velocity) / self.ka