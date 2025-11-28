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

    def physics_step(self, Va, Vb, Vc, Tload, Ia, Ib, Ic):
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

        # --- Calculate Id/Iq from Measured Currents (Ia, Ib, Ic) ---
        # Clarke
        I_alpha = Ia
        I_beta = (Ia + 2.0*Ib) / math.sqrt(3.0)
        
        # Park
        Id_meas = I_alpha * cos_t + I_beta * sin_t
        Iq_meas = -I_alpha * sin_t + I_beta * cos_t

        # Constants that depend on speed (We)
        g11 = 1 - (self.Ts * (self.Rs / self.Ld))
        g12 = (We * self.Lq * self.Ts) / self.Ld
        g21 = -We * self.Ld * self.Ts / self.Lq
        g22 = 1 - self.Rs * self.Ts / self.Lq
        h11 = self.Ts / self.Ld
        h22 = self.Ts / self.Lq
        i2 = -We * self.Lambda_m * self.Ts / self.Lq

        # --- Calculate Back-EMF in DQ frame (PMSM Sinusoidal) ---
        # For PMSM: ed = 0, eq = We * Lambda_m
        ed = 0.0
        eq = We * self.Lambda_m

        # Calculate next current states based on Applied Voltages and MEASURED currents
        Id_next = g11 * Id_meas + g12 * Iq_meas + h11 * Vd_ref
        Iq_next = g21 * Id_meas + g22 * Iq_meas + h22 * Vq_ref + i2
        
        # Torque Calculation
        # # Te = 1.5 * P * (ed * Id + eq * Iq) / We
        # if abs(We) > 1e-3:
        #     Te = 1.5 * self.Npp * (ed * Id_next + eq * Iq_next) / We
        # else:
        #     # Fallback for low speed (avoid division by zero)
        #     # For PMSM this simplifies to 1.5 * P * Lambda_m * Iq
        #     Te = 1.5 * self.Npp * self.Lambda_m * Iq_next
        Te = 1.5 * self.Npp * Iq_next * (self.Lambda_m + (self.Ld - self.Lq) * Id_next)

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


