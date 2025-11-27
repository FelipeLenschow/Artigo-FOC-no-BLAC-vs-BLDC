import math
import numpy as np

class BLDCMotor:
    def __init__(self, Ts):
        # --- Motor Parameters ---
        self.Ts = Ts          # Simulation step size
        self.Npp = 21.0       # Pole pairs
        self.Rs = 4.485       # Stator Resistance
        self.Ld = 0.0548      # D-axis Inductance
        self.Lq = 0.0548      # Q-axis Inductance (Ld=Lq for surface magnets)
        self.Lambda_m = 0.201 # Magnet Flux Amplitude
        self.Bn = 0.0057      # Friction
        self.J = 0.1444       # Inertia
        self.Tc = 0.3006      # Coulomb Torque
        
        # --- State Variables ---
        self.Id = 0.0
        self.Iq = 0.0
        self.Wr = 0.0         # Mechanical Speed (rad/s)
        self.theta = 0.0      # Mechanical Position (rad)
        self.theta_e = 0.0    # Electrical Position (rad)

    def _trapezoidal_shape(self, theta):
        # Normalize to 0-2pi
        t = theta % (2 * math.pi)
        # 120 degrees flat top
        # Rising: 0 to pi/3 (0 to 60 deg) -> 0 to 1 ?? 
        # Standard trapezoidal usually has 120 deg flat top.
        # Let's assume a normalized function f(theta) with amplitude 1.
        
        # Simple implementation:
        # 0 - pi/6: rising from 0 to 1? No, usually aligned such that peak is at 90 deg (pi/2).
        # So flat top from 30 to 150 degrees (pi/6 to 5pi/6).
        
        # return math.sin(t)
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
        # ---------------------------------------------------------
        # BLDC MOTOR PHYSICS MODEL
        # ---------------------------------------------------------
        
        # Calculate Electrical Variables
        We = self.Npp * self.Wr       # Electrical Speed
        self.theta_e = self.Npp * self.theta
        # Wrap theta_e to 0-2pi
        self.theta_e = self.theta_e % (2 * math.pi)

        # --- Calculate Back-EMF in ABC frame ---
        # Amplitude of Back-EMF: E = We * Lambda_m (approx)
        E_mag = We * self.Lambda_m
        
        # Phase shifts
        # Phase shifts
        ea = -E_mag * self._trapezoidal_shape(self.theta_e)
        eb = -E_mag * self._trapezoidal_shape(self.theta_e - 2*math.pi/3)
        ec = -E_mag * self._trapezoidal_shape(self.theta_e + 2*math.pi/3)

        # --- Park Transform of Back-EMF (to get ed, eq) ---
        cos_t = math.cos(self.theta_e)
        sin_t = math.sin(self.theta_e)
        
        # Clarke of EMF
        e_alpha = (2/3) * (ea - 0.5*eb - 0.5*ec)
        e_beta  = (2/3) * (math.sqrt(3)/2 * (eb - ec))
        
        # Park of EMF
        ed = e_alpha * cos_t + e_beta * sin_t
        eq = -e_alpha * sin_t + e_beta * cos_t

        # --- Park Transform of Voltages (Vabc -> Vdq) ---
        cos_t_m = math.cos(self.theta_e - 2*math.pi/3)
        sin_t_m = math.sin(self.theta_e - 2*math.pi/3)
        cos_t_p = math.cos(self.theta_e + 2*math.pi/3)
        sin_t_p = math.sin(self.theta_e + 2*math.pi/3)

        Vd_ref = (2.0/3.0) * (Va * cos_t + Vb * cos_t_m + Vc * cos_t_p)
        Vq_ref = (2.0/3.0) * (-Va * sin_t - Vb * sin_t_m - Vc * sin_t_p)

        # --- Calculate Id/Iq from Measured Currents (Ia, Ib, Ic) ---
        # Clarke
        I_alpha = Ia
        I_beta = (Ia + 2.0*Ib) / math.sqrt(3.0)
        
        # Park
        Id_meas = I_alpha * cos_t + I_beta * sin_t
        Iq_meas = -I_alpha * sin_t + I_beta * cos_t

        # --- State Update using dq equations with explicit ed, eq ---
        # dId/dt = (1/Ld) * (Vd - Rs*Id + We*Lq*Iq - ed)
        # dIq/dt = (1/Lq) * (Vq - Rs*Iq - We*Ld*Id - eq)
        
        # Discretization (Forward Euler):
        # Id_next = Id + Ts * dId/dt
        
        dId = (1.0/self.Ld) * (Vd_ref - self.Rs*Id_meas + We*self.Lq*Iq_meas - ed)
        dIq = (1.0/self.Lq) * (Vq_ref - self.Rs*Iq_meas - We*self.Ld*Id_meas - eq)
        
        Id_next = Id_meas + self.Ts * dId
        Iq_next = Iq_meas + self.Ts * dIq
        
        # Torque Calculation
        # Te = 1.5 * P * (lambda_m * iq + (Ld-Lq)*id*iq) 
        # BUT for BLDC, Te is sum of phase torques: Te = (ea*ia + eb*ib + ec*ic) / Wr
        # Or in dq frame: Te = 1.5 * P * (ed*id + eq*iq) / We ??? 
        # Actually Power = 1.5 * (vd*id + vq*iq). 
        # Electromagnetic Power = 1.5 * (ed*id + eq*iq).
        # Te = Pem / (We/P) = 1.5 * P * (ed*Id + eq*Iq) / We.
        # Note: ed, eq already contain We factor.
        # Let's use the Power balance: Te * (We/P) = 1.5 * (ed*Id + eq*Iq)
        # Te = 1.5 * P * (ed*Id + eq*Iq) / We
        # If We is close to 0, use limit.
        
        if abs(We) > 1e-3:
            Te = 1.5 * self.Npp * (ed * Id_next + eq * Iq_next) / We
        else:
            # At low speed, use k_t * iq approx or just 0
            # For trapezoidal, flux is constant at flat top.
            # Let's approximate with standard torque equation for safety at low speed
            Te = 1.5 * self.Npp * self.Lambda_m * Iq_next

        
        # Mechanical Dynamics (Euler Integration)
        # Handle Coulomb friction direction
        Tc_dir = self.Tc if self.Wr > 0 else (-self.Tc if self.Wr < 0 else 0)
        
        accel = (Te - Tload - (self.Bn * self.Wr) - Tc_dir) / self.J
        self.Wr += accel * self.Ts
        
        # Position Integration
        self.theta += self.Wr * self.Ts
        self.theta = self.theta % (2*math.pi) # Wrap mechanical angle

        # Save for sensing 
        self.Id = Id_next
        self.Iq = Iq_next

        return Te
