import math

class Sensors:
    def __init__(self):
        pass

    def measure(self, motor, RPMref):
        cos_t = math.cos(motor.theta_e)
        sin_t = math.sin(motor.theta_e)
        
        I_alpha = motor.Id * cos_t - motor.Iq * sin_t
        I_beta  = motor.Id * sin_t + motor.Iq * cos_t
        
        Ia = I_alpha
        Ib = -0.5 * I_alpha + (math.sqrt(3)/2.0) * I_beta
        Ic = -0.5 * I_alpha - (math.sqrt(3)/2.0) * I_beta
        
        theta_e = motor.theta_e
        
        Wr_meas = motor.Wr
        
        return Ia, Ib, Ic, theta_e, Wr_meas
