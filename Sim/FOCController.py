import math

class FOCController:
    def __init__(self, Ts, Imax=8.0):
        self.Ts = Ts
        self.Imax = Imax
        
        self.Kps = 1.25
        self.Kis = 55.0
        self.KpId = 119.0
        self.KiId = 4015.0
        self.KpIq = 119.0
        self.KiIq = 4015.0
        
        self.Ui_s = 0.0
        self.Ui_Id = 0.0
        self.Ui_Iq = 0.0

    def control_step(self, RPMref, Wr, Ia, Ib, Ic, theta_e, Vbus):
        I_alpha = Ia
        I_beta = (Ia + 2.0*Ib) / math.sqrt(3.0)
        
        cos_t = math.cos(theta_e)
        sin_t = math.sin(theta_e)
        
        Id = I_alpha * cos_t + I_beta * sin_t
        Iq = -I_alpha * sin_t + I_beta * cos_t
        
        error_speed = (RPMref * 2 * math.pi / 60.0) - Wr
        
        Up_s = self.Kps * error_speed
        Ui_s_next = self.Ui_s + (self.Kis * self.Ts * error_speed)
        
        Iq_ref = Up_s + Ui_s_next
        self.Ui_s = Ui_s_next 
        
        Id_ref = 0.0

        err_Iq = Iq_ref - Iq
        Up_Iq = self.KpIq * err_Iq
        Ui_Iq_next = self.Ui_Iq + (self.KiIq * self.Ts * err_Iq)
        Vq_ref = Up_Iq + Ui_Iq_next
        self.Ui_Iq = Ui_Iq_next
        
        err_Id = Id_ref - Id
        Up_Id = self.KpId * err_Id
        Ui_Id_next = self.Ui_Id + (self.KiId * self.Ts * err_Id)
        Vd_ref = Up_Id + Ui_Id_next
        self.Ui_Id = Ui_Id_next

        cos_t = math.cos(theta_e)
        sin_t = math.sin(theta_e)
        
        Va_ref = cos_t * Vd_ref - sin_t * Vq_ref
        Vb_ref = math.cos(theta_e - 2*math.pi/3) * Vd_ref - math.sin(theta_e - 2*math.pi/3) * Vq_ref
        Vc_ref = math.cos(theta_e + 2*math.pi/3) * Vd_ref - math.sin(theta_e + 2*math.pi/3) * Vq_ref

        V_mag = math.sqrt(Vd_ref**2 + Vq_ref**2)
        max_V = Vbus / math.sqrt(3)
        
        if V_mag > max_V:
            ratio = max_V / V_mag
            Vd_ref *= ratio
            Vq_ref *= ratio
            
        return Va_ref, Vb_ref, Vc_ref
