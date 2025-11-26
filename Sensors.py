import math

class Sensors:
    def __init__(self):
        pass

    def measure(self, motor, RPMref):
        # ---------------------------------------------------------
        # SENSOR MODEL
        # ---------------------------------------------------------
        
        # 1. Measure Currents (Ia, Ib, Ic)
        # Calculate Ia, Ib, Ic from motor states (Id, Iq, theta_e)
        
        # Inverse Park Transform (dq -> alpha, beta)
        cos_t = math.cos(motor.theta_e)
        sin_t = math.sin(motor.theta_e)
        
        I_alpha = motor.Id * cos_t - motor.Iq * sin_t
        I_beta  = motor.Id * sin_t + motor.Iq * cos_t
        
        # Inverse Clarke (alpha, beta -> abc)
        Ia = I_alpha
        Ib = -0.5 * I_alpha + (math.sqrt(3)/2.0) * I_beta
        Ic = -0.5 * I_alpha - (math.sqrt(3)/2.0) * I_beta
        
        # 2. Measure Position (theta_e)
        # Assuming perfect position sensor for FOC
        theta_e = motor.theta_e
        
        # 3. Measure Speed
        # Reverting to actual speed measurement to fix feedback loop
        Wr_meas = motor.Wr
        
        return Ia, Ib, Ic, theta_e, Wr_meas
