import math

class FOCController:
    def __init__(self, Ts, Imax=8.0):
        self.Ts = Ts
        self.Imax = Imax
        
        # --- Controller Parameters ---
        self.Kps = 1          # Speed P
        self.Kis = 55.0       # Speed I
        self.KpId = 119.0     # Id P
        self.KiId = 4015.0    # Id I
        self.KpIq = 119.0     # Iq P
        self.KiIq = 4015.0    # Iq I
        
        # --- Integrator States for PI Controllers ---
        self.Ui_s = 0.0
        self.Ui_Id = 0.0
        self.Ui_Iq = 0.0

    def control_step(self, RPMref, Wr, Ia, Ib, Ic, theta_e, Vbus):
        # ---------------------------------------------------------
        # FIELD ORIENTED CONTROL (FOC)
        # ---------------------------------------------------------
        
        # --- Clarke Transform (abc -> alpha, beta) ---
        # I_alpha = Ia
        # I_beta = (1/sqrt(3)) * (Ia + 2*Ib)  <-- Standard Clarke
        # Or: I_beta = (Ia + 2*Ib) / sqrt(3)
        
        I_alpha = Ia
        I_beta = (Ia + 2.0*Ib) / math.sqrt(3.0)
        
        # --- Park Transform (alpha, beta -> dq) ---
        cos_t = math.cos(theta_e)
        sin_t = math.sin(theta_e)
        
        Id = I_alpha * cos_t + I_beta * sin_t
        Iq = -I_alpha * sin_t + I_beta * cos_t
        
        # --- Speed Controller (Outer Loop) ---
        error_speed = (RPMref * 2 * math.pi / 60.0) - Wr
        
        # PI Calc
        Up_s = self.Kps * error_speed
        # Integral with Anti-windup clamping (Output limited to Imax)
        Ui_s_next = self.Ui_s + (self.Kis * self.Ts * error_speed)
        
        Iq_ref_unlimited = Up_s + Ui_s_next
        
        # Saturation / Clamp
        Iq_ref = max(-self.Imax, min(self.Imax, Iq_ref_unlimited))
        
        # Back-calculation / Anti-windup decision
        if Iq_ref == Iq_ref_unlimited:
            self.Ui_s = Ui_s_next
        else:
            # If saturated, do not update integral (simple anti-windup)
            pass 
        
        Id_ref = 0.0 # MTPA would go here, 0 for surface PMSM

        # --- Current Controllers (Inner Loops) ---
        
        # Iq Loop
        err_Iq = Iq_ref - Iq
        Up_Iq = self.KpIq * err_Iq
        Ui_Iq_next = self.Ui_Iq + (self.KiIq * self.Ts * err_Iq)
        Vq_ref = Up_Iq + Ui_Iq_next # + Decoupling (omitted as it was 0)
        self.Ui_Iq = Ui_Iq_next # Simplified update
        
        # Id Loop
        err_Id = Id_ref - Id
        Up_Id = self.KpId * err_Id
        Ui_Id_next = self.Ui_Id + (self.KiId * self.Ts * err_Id)
        Vd_ref = Up_Id + Ui_Id_next # - Decoupling (omitted as it was 0)
        self.Ui_Id = Ui_Id_next

        # ---------------------------------------------------------
        # INVERSE PARK & SVPWM
        # ---------------------------------------------------------
        
        cos_t = math.cos(theta_e)
        sin_t = math.sin(theta_e)
        
        # Inverse Park
        Va_ref = cos_t * Vd_ref - sin_t * Vq_ref
        Vb_ref = math.cos(theta_e - 2*math.pi/3) * Vd_ref - math.sin(theta_e - 2*math.pi/3) * Vq_ref
        Vc_ref = math.cos(theta_e + 2*math.pi/3) * Vd_ref - math.sin(theta_e + 2*math.pi/3) * Vq_ref

        # SVPWM Min-Max Injection (from C code logic)
        # Note: In the original simulation, this was calculated but not explicitly used for Vd/Vq modification
        # except for the Vbus limitation below.
        
        # Voltage Saturation based on Vbus (Limit circle)
        V_mag = math.sqrt(Vd_ref**2 + Vq_ref**2)
        max_V = Vbus / math.sqrt(3) # Max phase voltage with SVPWM
        
        if V_mag > max_V:
            ratio = max_V / V_mag
            Vd_ref *= ratio
            Vq_ref *= ratio
            
        return Va_ref, Vb_ref, Vc_ref
