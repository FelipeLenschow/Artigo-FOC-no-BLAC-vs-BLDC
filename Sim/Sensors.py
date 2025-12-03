import math
import Transforms

class Sensors:
    def __init__(self):
        pass

    def measure(self, motor, RPMref):
        theta_e = motor.theta_e
        
        Ia, Ib, Ic = Transforms.dq_to_abc(motor.Id, motor.Iq, theta_e)
        
        Wr_meas = motor.Wr
        
        return Ia, Ib, Ic, theta_e, Wr_meas
