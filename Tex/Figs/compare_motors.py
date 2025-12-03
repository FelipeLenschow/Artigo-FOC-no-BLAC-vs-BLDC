import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import os

# Go up two levels from 'Tex/Figs' to 'Sim'
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'Sim'))

from BLACMotor import BLACMotor
from BLDCMotor import BLDCMotor
from FOCController import FOCController
from Inverter import Inverter
from Sensors import Sensors

def run_simulation(motor_type):
    # Simulation Parameters
    Ts = 1e-4
    t_end = 1.0
    
    if motor_type == 'BLAC':
        motor = BLACMotor(Ts)
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
    
    # Enable LaTeX rendering and use Times font (matches IEEEtran)
    plt.rcParams.update({
        "text.usetex": True,
        "font.family": "serif",
        "font.serif": ["Times New Roman"],
        "text.latex.preamble": r"\usepackage{mathptmx}",
    })
    
    # Font sizes
    TITLE_SIZE = 22
    LABEL_SIZE = 18
    LEGEND_SIZE = 10
    
    fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)

    # Plot 1: Speed
    ax1.plot(pmsm_data['time'], pmsm_data['rpm_ref'], 'k--', label='Referência', linewidth=1.5, alpha=0.6)
    ax1.plot(pmsm_data['time'], pmsm_data['rpm_act'], 'b-', label='Velocidade BLAC', linewidth=1.5)
    ax1.plot(bldc_data['time'], bldc_data['rpm_act'], 'r-', label='Velocidade BLDC', linewidth=1.5)
    ax1.set_ylabel('Velocidade (RPM)', fontsize=LABEL_SIZE)
    ax1.set_title('Comparação de Resposta de Velocidade', fontsize=TITLE_SIZE)
    ax1.legend(loc='upper right', fontsize=LEGEND_SIZE)
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.tick_params(axis='both', which='major', labelsize=10)

    # Plot 2: Torque
    ax2.plot(pmsm_data['time'], pmsm_data['Tload'], 'k--', label='Torque de Carga', linewidth=1.5, alpha=0.6)
    ax2.plot(pmsm_data['time'], pmsm_data['Te'], 'b-', label='Torque BLAC', linewidth=1.5)
    ax2.plot(bldc_data['time'], bldc_data['Te'], 'r-', label='Torque BLDC', linewidth=1.5, alpha=0.8)
    ax2.set_ylabel('Torque (Nm)', fontsize=LABEL_SIZE)
    ax2.set_xlabel('Tempo (s)', fontsize=LABEL_SIZE)
    ax2.set_title('Comparação de Resposta de Torque', fontsize=TITLE_SIZE)
    ax2.legend(loc='upper right', fontsize=LEGEND_SIZE)
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.tick_params(axis='both', which='major', labelsize=10)
    
    plt.tight_layout()
    output_path1 = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'comparison_speed_torque.png')
    plt.savefig(output_path1, dpi=300)
    print(f"Figure saved to {output_path1}")

    fig2, (ax3, ax4) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)
    
    # Plot 3: Iq Current
    ax3.plot(pmsm_data['time'], pmsm_data['Iq'], 'b-', label='BLAC $I_q$', linewidth=1.5)
    ax3.plot(bldc_data['time'], bldc_data['Iq'], 'r-', label='BLDC $I_q$', linewidth=1.5, alpha=0.8)
    ax3.set_ylabel('Corrente $I_q$ (A)', fontsize=LABEL_SIZE)
    ax3.set_title('Comparação da Corrente $I_q$', fontsize=TITLE_SIZE)
    ax3.legend(loc='upper right', fontsize=LEGEND_SIZE)
    ax3.grid(True, linestyle='--', alpha=0.7)
    ax3.tick_params(axis='both', which='major', labelsize=10)

    # Plot 4: Id Current
    ax4.plot(pmsm_data['time'], pmsm_data['Id'], 'b-', label='BLAC $I_d$', linewidth=1.5)
    ax4.plot(bldc_data['time'], bldc_data['Id'], 'r-', label='BLDC $I_d$', linewidth=1.5, alpha=0.8)
    ax4.set_ylabel('Corrente $I_d$ (A)', fontsize=LABEL_SIZE)
    ax4.set_xlabel('Tempo (s)', fontsize=LABEL_SIZE)
    ax4.set_title('Comparação da Corrente $I_d$', fontsize=TITLE_SIZE)
    ax4.legend(loc='upper right', fontsize=LEGEND_SIZE)
    ax4.grid(True, linestyle='--', alpha=0.7)
    ax4.tick_params(axis='both', which='major', labelsize=10)

    plt.tight_layout()
    output_path2 = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'comparison_currents.png')
    plt.savefig(output_path2, dpi=300)
    print(f"Figure saved to {output_path2}")

if __name__ == "__main__":
    pmsm_results = run_simulation('BLAC')
    bldc_results = run_simulation('BLDC')
    plot_comparisons(pmsm_results, bldc_results)
