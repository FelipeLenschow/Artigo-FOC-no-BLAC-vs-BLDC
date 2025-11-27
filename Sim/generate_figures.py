import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import os

# Ensure we can import from the current directory
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from PMSMMotor import PMSMMotor
from FOCController import FOCController
from Inverter import Inverter
from Sensors import Sensors

def run_simulation_and_save_plot():
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

    print("Starting Simulation for Figure Generation...")

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
        Te = motor.physics_step(Va, Vb, Vc, Tload, Ia, Ib, Ic)

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
    # Use a style suitable for papers
    plt.style.use('default')
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

    # Plot 1: Speed
    ax1.plot(data['time'], data['rpm_ref'], 'r--', label='Reference Speed', linewidth=1.5)
    ax1.plot(data['time'], data['rpm_act'], 'b-', label='Actual Speed', linewidth=1.5)
    ax1.set_ylabel('Speed (RPM)')
    ax1.set_title('PMSM FOC Simulation Results')
    ax1.legend(loc='upper right')
    ax1.grid(True, linestyle='--', alpha=0.7)

    # Plot 2: Torque
    ax2.plot(data['time'], data['Te'], 'g-', label='Electromagnetic Torque', linewidth=1.5)
    ax2.plot(data['time'], data['Tload'], 'k--', label='Load Torque', linewidth=1.5)
    ax2.set_ylabel('Torque (Nm)')
    ax2.legend(loc='upper right')
    ax2.grid(True, linestyle='--', alpha=0.7)
    
    # Plot 3: Currents
    ax3.plot(data['time'], data['Iq'], 'c-', label='$I_q$', linewidth=1.5)
    ax3.plot(data['time'], data['Id'], 'm-', label='$I_d$', linewidth=1.5)
    ax3.set_ylabel('Current (A)')
    ax3.set_xlabel('Time (s)')
    ax3.legend(loc='upper right')
    ax3.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    
    # Save to Tex directory
    output_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'Tex', 'sim_results.png')
    plt.savefig(output_path, dpi=300)
    print(f"Figure saved to {output_path}")

if __name__ == "__main__":
    run_simulation_and_save_plot()
