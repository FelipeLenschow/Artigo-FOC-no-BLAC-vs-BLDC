import math
import Transforms

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
        
        Id, Iq = Transforms.abc_to_dq(Ia, Ib, Ic, theta_e)

        # Controlador de velocidade        
        error_speed = (RPMref * 2 * math.pi / 60.0) - Wr
        Up_s = self.Kps * error_speed
        Ui_s_next = self.Ui_s + (self.Kis * self.Ts * error_speed)
        Iq_ref = Up_s + Ui_s_next
        self.Ui_s = Ui_s_next 
        
        Id_ref = 0.0

        # Controlador de corrente Iq
        err_Iq = Iq_ref - Iq
        Up_Iq = self.KpIq * err_Iq
        Ui_Iq_next = self.Ui_Iq + (self.KiIq * self.Ts * err_Iq)
        Vq_ref = Up_Iq + Ui_Iq_next
        self.Ui_Iq = Ui_Iq_next
        
        # Controlador de corrente Id
        err_Id = Id_ref - Id
        Up_Id = self.KpId * err_Id
        Ui_Id_next = self.Ui_Id + (self.KiId * self.Ts * err_Id)
        Vd_ref = Up_Id + Ui_Id_next
        self.Ui_Id = Ui_Id_next

        Va_ref, Vb_ref, Vc_ref = Transforms.dq_to_abc(Vd_ref, Vq_ref, theta_e)
            
        return Va_ref, Vb_ref, Vc_ref
