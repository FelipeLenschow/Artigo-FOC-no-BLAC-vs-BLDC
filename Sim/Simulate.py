import numpy as np
import matplotlib.pyplot as plt
import math
from PMSMMotor import PMSMMotor
from FOCController import FOCController
from Inverter import Inverter
from Sensors import Sensors

# --- Main Execution ---
if __name__ == "__main__":
    # Simulation Parameters
    Ts = 1e-4
    t_end = 1.0
    
    # Initialize Modules
    motor = PMSMMotor(Ts)
    controller = FOCController(Ts)
    inverter = Inverter()
    sensors = Sensors()
    
    # Time settings
    num_steps = int(t_end / Ts)

    # Storage for plotting
    history = {
        'time': np.zeros(num_steps),
        'rpm_ref': np.zeros(num_steps),
        'rpm_act': np.zeros(num_steps),
        'Iq': np.zeros(num_steps),
        'Id': np.zeros(num_steps),
        'Te': np.zeros(num_steps),
        'Tload': np.zeros(num_steps),
        'Vbus': np.zeros(num_steps)
    }

    print("Starting Simulation...")

    t = 0.0
    for k in range(num_steps):
        # Update time
        t = k * Ts
        history['time'][k] = t

        # ---------------------------------------------------------
        # 1. INPUTS & PROFILE
        # ---------------------------------------------------------
        RPMref = 40.0
        Tload = 0.0
        
        if t > 0.2:
            Tload = 20.0
        if t > 0.4:
            RPMref = 80.0
        if t > 0.6:
            RPMref = 40.0
        if t > 0.8:
            Tload = 0.0
            
        V_bus = 311.0 

        # ---------------------------------------------------------
        # 2. SENSORS STEP
        # ---------------------------------------------------------
        Ia, Ib, Ic, theta_e, Wr_meas = sensors.measure(motor, RPMref)

        # ---------------------------------------------------------
        # 3. CONTROL STEP
        # ---------------------------------------------------------
        Va_ref, Vb_ref, Vc_ref = controller.control_step(
            RPMref, Wr_meas, Ia, Ib, Ic, theta_e, V_bus
        )

        # ---------------------------------------------------------
        # 4. INVERTER STEP
        # ---------------------------------------------------------
        Va, Vb, Vc = inverter.step(Va_ref, Vb_ref, Vc_ref, V_bus)

        # ---------------------------------------------------------
        # 5. MOTOR PHYSICS STEP
        # ---------------------------------------------------------
        Te = motor.physics_step(Va, Vb, Vc, Tload)

        # ---------------------------------------------------------
        # 6. DATA LOGGING
        # ---------------------------------------------------------
        history['rpm_ref'][k] = RPMref
        history['rpm_act'][k] = motor.Wr * 60 / (2*math.pi)
        history['Iq'][k] = motor.Iq
        history['Id'][k] = motor.Id
        history['Te'][k] = Te
        history['Tload'][k] = Tload
        history['Vbus'][k] = V_bus
        
    data = history

    # --- Plotting ---
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    # Plot 1: Speed
    ax1.plot(data['time'], data['rpm_ref'], 'r--', label='RPM Ref')
    ax1.plot(data['time'], data['rpm_act'], 'b-', label='RPM Actual')
    ax1.set_ylabel('Speed (RPM)')
    ax1.set_title('PMSM FOC Simulation')
    ax1.legend()
    ax1.grid(True)

    # Plot 2: Torque
    ax2.plot(data['time'], data['Te'], 'g-', label='Electromagnetic Torque')
    ax2.plot(data['time'], data['Tload'], 'k--', label='Load Torque')
    ax2.set_ylabel('Torque (Nm)')
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: Currents and Voltage Input
    ax3.plot(data['time'], data['Iq'], 'c-', label='Iq (A)')
    ax3.plot(data['time'], data['Id'], 'm-', label='Id (A)')
    # Scaling Vbus to fit on plot for visualization
    ax3.plot(data['time'], data['Vbus']/10, 'y-', alpha=0.3, label='Vbus Input / 10 (V)')
    ax3.set_ylabel('Current (A)')
    ax3.set_xlabel('Time (s)')
    ax3.legend()
    ax3.grid(True)

    plt.tight_layout()
    plt.show()