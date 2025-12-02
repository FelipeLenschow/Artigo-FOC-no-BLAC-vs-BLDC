import math
import numpy as np
import matplotlib.pyplot as plt
import os
import sys

# Ensure we can import from current directory if needed (though not strictly necessary for just plotting if no local imports)
# Also add Sim directory just in case
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'Sim'))

def trapezoidal_shape(theta):
    t = theta % (2 * math.pi)
    pi = math.pi

    if t < pi/6:
        return t * (6/pi) # 0 to 1
    elif t < 5*pi/6:
        return 1.0
    elif t < 7*pi/6:
        return 1.0 - (t - 5*pi/6) * (6/pi) # 1 to -1
    elif t < 11*pi/6:
        return -1.0
    else:
        return -1.0 + (t - 11*pi/6) * (6/pi) # -1 to 0

def dq0_transform(va, vb, vc, theta):
    v_alpha = (2/3) * (va - 0.5*vb - 0.5*vc)
    v_beta  = (2/3) * (math.sqrt(3)/2 * (vb - vc))
    
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    
    vd = v_alpha * cos_t + v_beta * sin_t
    vq = -v_alpha * sin_t + v_beta * cos_t
    
    return vd, vq

def generate_plot():
    theta_e = np.linspace(0, 4*np.pi, 1000)
    
    ea_bldc_list = []
    eb_bldc_list = []
    ec_bldc_list = []
    ea_blac_list = []
    eb_blac_list = []
    ec_blac_list = []
    
    ed_bldc_list = []
    eq_bldc_list = []
    
    ed_blac_list = []
    eq_blac_list = []

    E_mag = 1.0 # Normalized amplitude

    for t in theta_e:
        val_a_bldc = -E_mag * trapezoidal_shape(t)
        val_b_bldc = -E_mag * trapezoidal_shape(t - 2*math.pi/3)
        val_c_bldc = -E_mag * trapezoidal_shape(t + 2*math.pi/3)
        
        ea_bldc_list.append(val_a_bldc)
        eb_bldc_list.append(val_b_bldc)
        ec_bldc_list.append(val_c_bldc)
        
        vd_bldc, vq_bldc = dq0_transform(val_a_bldc, val_b_bldc, val_c_bldc, t)
        ed_bldc_list.append(vd_bldc)
        eq_bldc_list.append(vq_bldc)

        val_a_blac = -E_mag * math.sin(t)
        val_b_blac = -E_mag * math.sin(t - 2*math.pi/3)
        val_c_blac = -E_mag * math.sin(t + 2*math.pi/3)
        
        ea_blac_list.append(val_a_blac)
        eb_blac_list.append(val_b_blac)
        ec_blac_list.append(val_c_blac)
        
        vd_blac, vq_blac = dq0_transform(val_a_blac, val_b_blac, val_c_blac, t)
        ed_blac_list.append(vd_blac)
        eq_blac_list.append(vq_blac)

    plt.figure(figsize=(10, 5))
    
    # Plot phases B and C in light gray (background)
    plt.plot(theta_e, eb_bldc_list, linestyle='-', linewidth=1, color='lightgray', zorder=1)
    plt.plot(theta_e, ec_bldc_list, linestyle='-', linewidth=1, color='lightgray', zorder=1)
    plt.plot(theta_e, eb_blac_list, linestyle='--', linewidth=1, color='lightgray', zorder=1)
    plt.plot(theta_e, ec_blac_list, linestyle='--', linewidth=1, color='lightgray', zorder=1)
    
    # Plot main phases and dq components
    plt.plot(theta_e, ea_bldc_list, label=r'$e_{a,BLDC}$', linestyle='-', linewidth=2, color='blue', zorder=2)
    plt.plot(theta_e, ed_bldc_list, label=r'$e_{d,BLDC}$', linestyle='-', linewidth=2, color='red', zorder=2)
    plt.plot(theta_e, eq_bldc_list, label=r'$e_{q,BLDC}$', linestyle='-', linewidth=2, color='green', zorder=2)
    plt.plot(theta_e, ea_blac_list, label=r'$e_{a,BLAC}$', linestyle='--', linewidth=2, color='cyan', zorder=2)
    plt.plot(theta_e, ed_blac_list, label=r'$e_{d,BLAC}$', linestyle='--', linewidth=2, color='orange', zorder=2)
    plt.plot(theta_e, eq_blac_list, label=r'$e_{q,BLAC}$', linestyle='--', linewidth=2, color='lime', zorder=2)
    
    plt.xlabel('Ângulo elétrico (rad)')
    plt.ylabel('Amplitude (Normalizada)')
    plt.title('Comparação de Back-EMF: BLDC vs BLAC (ABC e DQ Frames)')
    plt.legend(loc='upper right', ncol=2)
    plt.grid(True)

    plt.tight_layout()
    
    plt.tight_layout()
    
    # Save to the current directory (Tex/Figs)
    output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'back_emf_plot.png')
    plt.savefig(output_path, dpi=300)
    print(f"Plot saved to {output_path}")

if __name__ == "__main__":
    generate_plot()
