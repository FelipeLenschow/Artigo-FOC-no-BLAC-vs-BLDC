import math
import Transforms

class BLDCMotor:
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

    def _trapezoidal_shape(self, theta):
        t = theta % (2 * math.pi)
        pi = math.pi

        if t < pi/6:
            return t * (6/pi) # 0 to 1
        elif t < 5*pi/6:
            return 1.0
        elif t < 7*pi/6:
            return 1.0 - (t - 5*pi/6) * (6/pi) # 1 to -1
        elif t < 11*pi/6:
            return -1.0
        else:
            return -1.0 + (t - 11*pi/6) * (6/pi) # -1 to 0

    def physics_step(self, Va, Vb, Vc, Tload, Ia, Ib, Ic):
        We = self.Npp * self.Wr
        self.theta_e = self.Npp * self.theta
        self.theta_e = self.theta_e % (2 * math.pi)

        E_mag = We * self.Lambda_m
        
        ea = -E_mag * self._trapezoidal_shape(self.theta_e)
        eb = -E_mag * self._trapezoidal_shape(self.theta_e - 2*math.pi/3)
        ec = -E_mag * self._trapezoidal_shape(self.theta_e + 2*math.pi/3)
        
        ed, eq = Transforms.abc_to_dq(ea, eb, ec, self.theta_e)
        Vd_ref, Vq_ref = Transforms.abc_to_dq(Va, Vb, Vc, self.theta_e)
        Id_meas, Iq_meas = Transforms.abc_to_dq(Ia, Ib, Ic, self.theta_e)

        dId = (1.0/self.Ld) * (Vd_ref - self.Rs*Id_meas + We*self.Lq*Iq_meas - ed)
        dIq = (1.0/self.Lq) * (Vq_ref - self.Rs*Iq_meas - We*self.Ld*Id_meas - eq)
        
        Id_next = Id_meas + self.Ts * dId
        Iq_next = Iq_meas + self.Ts * dIq
        
        if abs(We) > 1e-3:
            Te = 1.5 * self.Npp * (ed * Id_next + eq * Iq_next) / We + \
            1.5 * self.Npp * (self.Ld - self.Lq) * Id_next * Iq_next
        else:
            Te = 1.5 * self.Npp * self.Lambda_m * Iq_next
        
        Tc_dir = self.Tc if self.Wr > 0 else (-self.Tc if self.Wr < 0 else 0)
        
        accel = (Te - Tload - (self.Bn * self.Wr) - Tc_dir) / self.J
        self.Wr += accel * self.Ts
        
        self.theta += self.Wr * self.Ts
        self.theta = self.theta % (2*math.pi)

        self.Id = Id_next
        self.Iq = Iq_next

        return Te
