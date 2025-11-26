import math
import numpy as np

class PMSMMotor:
    def __init__(self, Ts):
        # --- Motor Parameters ---
        self.Ts = Ts          # Simulation step size
        self.Npp = 21.0       # Pole pairs
        self.Rs = 4.485       # Stator Resistance
        self.Ld = 0.0548      # D-axis Inductance
        self.Lq = 0.0548      # Q-axis Inductance
        self.Lambda_m = 0.201 # Magnet Flux
        self.Bn = 0.0057      # Friction
        self.J = 0.1444       # Inertia
        self.Tc = 0.3006      # Coulomb Torque
        
        # --- State Variables ---
        self.Id = 0.0
        self.Iq = 0.0
        self.Wr = 0.0         # Mechanical Speed (rad/s)
        self.theta = 0.0      # Mechanical Position (rad)
        self.theta_e = 0.0    # Electrical Position (rad)

    def physics_step(self, Va, Vb, Vc, Tload):
        # ---------------------------------------------------------
        # MOTOR PHYSICS MODEL (The "Plant")
        # ---------------------------------------------------------
        
        # Calculate Electrical Variables
        We = self.Npp * self.Wr       # Electrical Speed
        self.theta_e = self.Npp * self.theta
        # Wrap theta_e to 0-2pi
        self.theta_e = self.theta_e % (2 * math.pi)

        # --- Park Transform (Vabc -> Vdq) ---
        cos_t = math.cos(self.theta_e)
        sin_t = math.sin(self.theta_e)
        cos_t_m = math.cos(self.theta_e - 2*math.pi/3)
        sin_t_m = math.sin(self.theta_e - 2*math.pi/3)
        cos_t_p = math.cos(self.theta_e + 2*math.pi/3)
        sin_t_p = math.sin(self.theta_e + 2*math.pi/3)

        Vd_ref = (2.0/3.0) * (Va * cos_t + Vb * cos_t_m + Vc * cos_t_p)
        Vq_ref = (2.0/3.0) * (-Va * sin_t - Vb * sin_t_m - Vc * sin_t_p)

        # Constants that depend on speed (We)
        g1 = 1 - (self.Ts * (self.Rs / self.Ld))
        g2 = (We * self.Lq * self.Ts) / self.Ld
        g3 = -We * self.Ld * self.Ts / self.Lq
        g4 = 1 - self.Rs * self.Ts / self.Lq
        h1 = self.Ts / self.Ld
        h2 = self.Ts / self.Lq
        i2 = -We * self.Lambda_m * self.Ts / self.Lq

        # Calculate next current states based on Applied Voltages
        Id_next = g1 * self.Id + g2 * self.Iq + h1 * Vd_ref
        Iq_next = g3 * self.Id + g4 * self.Iq + h2 * Vq_ref + i2
        
        self.Id = Id_next
        self.Iq = Iq_next
        
        # Torque Calculation
        Te = 1.5 * self.Npp * (self.Lambda_m * self.Iq + (self.Ld - self.Lq) * self.Id * self.Iq)
        
        # Mechanical Dynamics (Euler Integration)
        # Handle Coulomb friction direction
        Tc_dir = self.Tc if self.Wr > 0 else (-self.Tc if self.Wr < 0 else 0)
        
        accel = (Te - Tload - (self.Bn * self.Wr) - Tc_dir) / self.J
        self.Wr += accel * self.Ts
        
        # Position Integration
        self.theta += self.Wr * self.Ts
        self.theta = self.theta % (2*math.pi) # Wrap mechanical angle

        return Te


