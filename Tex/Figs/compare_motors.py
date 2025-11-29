import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import os

# Go up two levels from 'Tex/Figs' to 'Sim'
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'Sim'))

from PMSMMotor import PMSMMotor
from BLDCMotor import BLDCMotor
from FOCController import FOCController
from Inverter import Inverter
from Sensors import Sensors

def run_simulation(motor_type):
    # Simulation Parameters
    Ts = 1e-4
    t_end = 1.0
    
    if motor_type == 'PMSM':
        motor = PMSMMotor(Ts)
    elif motor_type == 'BLDC':
        motor = BLDCMotor(Ts)
    else:
        raise ValueError("Invalid motor type")
        
    controller = FOCController(Ts)
    inverter = Inverter()
    sensors = Sensors()
    
    num_steps = int(t_end / Ts)

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

    print(f"Starting Simulation for {motor_type}...")

    t = 0.0
    for k in range(num_steps):
        # Update time
        t = k * Ts
        history['time'][k] = t

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

        Ia, Ib, Ic, theta_e, Wr_meas = sensors.measure(motor, RPMref)

        Va_ref, Vb_ref, Vc_ref = controller.control_step(
            RPMref, Wr_meas, Ia, Ib, Ic, theta_e, V_bus
        )

        Va, Vb, Vc = inverter.step(Va_ref, Vb_ref, Vc_ref, V_bus)

        Te = motor.physics_step(Va, Vb, Vc, Tload, Ia, Ib, Ic)

        history['rpm_ref'][k] = RPMref
        history['rpm_act'][k] = motor.Wr * 60 / (2*math.pi)
        history['Iq'][k] = motor.Iq
        history['Id'][k] = motor.Id
        history['Te'][k] = Te
        history['Tload'][k] = Tload
        history['Vbus'][k] = V_bus
        
    return history

def plot_comparisons(pmsm_data, bldc_data):
    # Use a style suitable for papers
    plt.style.use('default')
    
    fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)

    # Plot 1: Speed
    ax1.plot(pmsm_data['time'], pmsm_data['rpm_ref'], 'k--', label='Reference', linewidth=1.5, alpha=0.6)
    ax1.plot(pmsm_data['time'], pmsm_data['rpm_act'], 'b-', label='PMSM Speed', linewidth=1.5)
    ax1.plot(bldc_data['time'], bldc_data['rpm_act'], 'r-', label='BLDC Speed', linewidth=1.5)
    ax1.set_ylabel('Speed (RPM)')
    ax1.set_title('Speed Response Comparison')
    ax1.legend(loc='upper right')
    ax1.grid(True, linestyle='--', alpha=0.7)

    # Plot 2: Torque
    ax2.plot(pmsm_data['time'], pmsm_data['Tload'], 'k--', label='Load Torque', linewidth=1.5, alpha=0.6)
    ax2.plot(pmsm_data['time'], pmsm_data['Te'], 'b-', label='PMSM Torque', linewidth=1.5)
    ax2.plot(bldc_data['time'], bldc_data['Te'], 'r-', label='BLDC Torque', linewidth=1.5, alpha=0.8)
    ax2.set_ylabel('Torque (Nm)')
    ax2.set_xlabel('Time (s)')
    ax2.set_title('Torque Response Comparison')
    ax2.legend(loc='upper right')
    ax2.grid(True, linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    output_path1 = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'comparison_speed_torque.png')
    plt.savefig(output_path1, dpi=300)
    print(f"Figure saved to {output_path1}")

    fig2, (ax3, ax4) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)
    
    # Plot 3: Iq Current
    ax3.plot(pmsm_data['time'], pmsm_data['Iq'], 'b-', label='PMSM $I_q$', linewidth=1.5)
    ax3.plot(bldc_data['time'], bldc_data['Iq'], 'r-', label='BLDC $I_q$', linewidth=1.5, alpha=0.8)
    ax3.set_ylabel('Current $I_q$ (A)')
    ax3.set_title('$I_q$ Current Comparison')
    ax3.legend(loc='upper right')
    ax3.grid(True, linestyle='--', alpha=0.7)

    # Plot 4: Id Current
    ax4.plot(pmsm_data['time'], pmsm_data['Id'], 'b-', label='PMSM $I_d$', linewidth=1.5)
    ax4.plot(bldc_data['time'], bldc_data['Id'], 'r-', label='BLDC $I_d$', linewidth=1.5, alpha=0.8)
    ax4.set_ylabel('Current $I_d$ (A)')
    ax4.set_xlabel('Time (s)')
    ax4.set_title('$I_d$ Current Comparison')
    ax4.legend(loc='upper right')
    ax4.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    output_path2 = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'comparison_currents.png')
    plt.savefig(output_path2, dpi=300)
    print(f"Figure saved to {output_path2}")

if __name__ == "__main__":
    pmsm_results = run_simulation('PMSM')
    bldc_results = run_simulation('BLDC')
    plot_comparisons(pmsm_results, bldc_results)
