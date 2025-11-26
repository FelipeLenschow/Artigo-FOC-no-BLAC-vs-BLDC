import math

class Inverter:
    def __init__(self):
        pass

    def step(self, Va_ref, Vb_ref, Vc_ref, Vbus):
        # ---------------------------------------------------------
        # INVERTER MODEL
        # ---------------------------------------------------------
        # Simple voltage source inverter model.
        # Limits the output phase voltages based on Vbus.
        # In a real inverter, this would involve PWM duty cycles.
        # Here we assume average voltage injection with saturation.

        # Max phase voltage (linear modulation limit for SVPWM)
        max_V = Vbus / math.sqrt(3)
        
        # Calculate magnitude of the requested voltage vector
        # (Assuming balanced 3-phase, we can estimate magnitude)
        # A more robust check is to convert back to alpha-beta or dq to check magnitude,
        # but here we can check individual phase limits or the vector sum.
        
        # Let's use the same logic as before: limit the vector magnitude.
        # To do this without dq, we can look at the peak.
        # Or simpler: just clamp individual phases to +/- Vbus/2 (DC link midpoint ref)
        # BUT, the previous logic used a circular limit on Vdq.
        
        # Let's stick to the previous logic's intent:
        # If we receive Vabc, we assume they are already appropriate.
        # However, we should enforce the physical limit of the bus.
        
        # For this simulation, let's assume the controller handles the circular limit (SVPWM),
        # and the inverter just hard-clamps if something goes wrong, or models the PWM effect.
        
        # Since the controller was doing the limiting, we can just pass through
        # or add a hard clamp for safety.
        
        # Let's add a simple clamp to +/- Vbus/2 for each phase relative to neutral point
        # (assuming ideal DC link utilization).
        
        limit = Vbus / 2.0
        
        Va = max(-limit, min(limit, Va_ref))
        Vb = max(-limit, min(limit, Vb_ref))
        Vc = max(-limit, min(limit, Vc_ref))
        
        return Va, Vb, Vc
