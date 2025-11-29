import math

class Inverter:
    def __init__(self):
        pass

    def step(self, Va_ref, Vb_ref, Vc_ref, Vbus):
        limit = Vbus / 2.0
        
        Va = max(-limit, min(limit, Va_ref))
        Vb = max(-limit, min(limit, Vb_ref))
        Vc = max(-limit, min(limit, Vc_ref))
        
        return Va, Vb, Vc
