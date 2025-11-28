import math
import numpy as np
import matplotlib.pyplot as plt

def trapezoidal_shape(theta):
    # Normalize to 0-2pi
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

def generate_plot():
    theta_e = np.linspace(0, 4*np.pi, 1000)
    ea = []
    eb = []
    ec = []
    ed = []
    eq = []

    E_mag = 1.0 # Normalized amplitude

    for t in theta_e:
        # Phase EMFs
        val_a = -E_mag * trapezoidal_shape(t)
        val_b = -E_mag * trapezoidal_shape(t - 2*math.pi/3)
        val_c = -E_mag * trapezoidal_shape(t + 2*math.pi/3)
        
        ea.append(val_a)
        eb.append(val_b)
        ec.append(val_c)

        # Park Transform
        cos_t = math.cos(t)
        sin_t = math.sin(t)
        
        # Clarke
        e_alpha = (2/3) * (val_a - 0.5*val_b - 0.5*val_c)
        e_beta  = (2/3) * (math.sqrt(3)/2 * (val_b - val_c))
        
        # Park
        val_d = e_alpha * cos_t + e_beta * sin_t
        val_q = -e_alpha * sin_t + e_beta * cos_t
        
        ed.append(val_d)
        eq.append(val_q)

    # Plotting
    # Plotting
    plt.figure(figsize=(8, 4))
    
    # Plot ABC Back-EMF
    plt.plot(theta_e, ea, label=r'$e_a$', linestyle='-', linewidth=3)
    plt.plot(theta_e, eb, label=r'$e_b$', linestyle='--', linewidth=2)
    plt.plot(theta_e, ec, label=r'$e_c$', linestyle='--', linewidth=2)
    
    # Plot DQ Back-EMF
    plt.plot(theta_e, ed, label=r'$e_d$', linewidth=2)
    plt.plot(theta_e, eq, label=r'$e_q$', linewidth=2)
    
    plt.xlabel('Electrical Angle (rad)')
    plt.ylabel('Amplitude (Normalized)')
    plt.title('Trapezoidal Back-EMF in ABC and DQ Frames')
    plt.legend(loc='upper right')
    plt.grid(True)

    plt.tight_layout()
    
    # Save to the Tex directory where paper.tex resides
    output_path = r'c:\Users\Felipe\OneDrive - UDESC Universidade do Estado de Santa Catarina\Mestrado\1 Semestre\Controle de Motores\Artigo\Tex\back_emf_plot.png'
    plt.savefig(output_path, dpi=300)
    print(f"Plot saved to {output_path}")

if __name__ == "__main__":
    generate_plot()
