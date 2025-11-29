import math
import numpy as np

class PMSMMotor:
    def __init__(self, Ts):
        self.Ts = Ts
        self.Npp = 21.0
        self.Rs = 4.485
        self.Ld = 0.0548
        self.Lq = 0.0548
        self.Lambda_m = 0.201
        self.Bn = 0.0057
        self.J = 0.1444
        self.Tc = 0.3006
        
        self.Id = 0.0
        self.Iq = 0.0
        self.Wr = 0.0
        self.theta = 0.0
        self.theta_e = 0.0

    def physics_step(self, Va, Vb, Vc, Tload, Ia, Ib, Ic):
        We = self.Npp * self.Wr
        self.theta_e = self.Npp * self.theta
        self.theta_e = self.theta_e % (2 * math.pi)

        cos_t = math.cos(self.theta_e)
        sin_t = math.sin(self.theta_e)
        cos_t_m = math.cos(self.theta_e - 2*math.pi/3)
        sin_t_m = math.sin(self.theta_e - 2*math.pi/3)
        cos_t_p = math.cos(self.theta_e + 2*math.pi/3)
        sin_t_p = math.sin(self.theta_e + 2*math.pi/3)

        Vd_ref = (2.0/3.0) * (Va * cos_t + Vb * cos_t_m + Vc * cos_t_p)
        Vq_ref = (2.0/3.0) * (-Va * sin_t - Vb * sin_t_m - Vc * sin_t_p)

        I_alpha = Ia
        I_beta = (Ia + 2.0*Ib) / math.sqrt(3.0)
        
        Id_meas = I_alpha * cos_t + I_beta * sin_t
        Iq_meas = -I_alpha * sin_t + I_beta * cos_t

        g11 = 1 - (self.Ts * (self.Rs / self.Ld))
        g12 = (We * self.Lq * self.Ts) / self.Ld
        g21 = -We * self.Ld * self.Ts / self.Lq
        g22 = 1 - self.Rs * self.Ts / self.Lq
        h11 = self.Ts / self.Ld
        h22 = self.Ts / self.Lq
        i2 = -We * self.Lambda_m * self.Ts / self.Lq

        ed = 0.0
        eq = We * self.Lambda_m

        Id_next = g11 * Id_meas + g12 * Iq_meas + h11 * Vd_ref
        Iq_next = g21 * Id_meas + g22 * Iq_meas + h22 * Vq_ref + i2
        
        Te = 1.5 * self.Npp * Iq_next * (self.Lambda_m + (self.Ld - self.Lq) * Id_next)

        Tc_dir = self.Tc if self.Wr > 0 else (-self.Tc if self.Wr < 0 else 0)
        
        accel = (Te - Tload - (self.Bn * self.Wr) - Tc_dir) / self.J
        self.Wr += accel * self.Ts
        
        self.theta += self.Wr * self.Ts
        self.theta = self.theta % (2*math.pi)

        self.Id = Id_next
        self.Iq = Iq_next

        return Te


