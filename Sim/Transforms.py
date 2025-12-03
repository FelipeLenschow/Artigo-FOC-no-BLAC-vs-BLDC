import math

def abc_to_dq(Xa, Xb, Xc, theta):
    # Clarke transform
    X_alpha = (2/3) * (Xa - 0.5 * Xb - 0.5 * Xc)
    X_beta = (2/3) * (math.sqrt(3)/2 * (Xb - Xc))
    
    # Park transform
    Xd = X_alpha * math.cos(theta) + X_beta * math.sin(theta)
    Xq = -X_alpha * math.sin(theta) + X_beta * math.cos(theta)
    
    # Direct Park Transform (abc -> dq)
    # Xd = 2/3 * (Xa * math.cos(theta) + Xb * math.cos(theta - 2*math.pi/3) + Xc * math.cos(theta + 2*math.pi/3))
    # Xq = 2/3 * (-Xa * math.sin(theta) - Xb * math.sin(theta - 2*math.pi/3) - Xc * math.sin(theta + 2*math.pi/3))
    
    return Xd, Xq

def dq_to_abc(Xd, Xq, theta):
    # Inverse Park transform
    X_alpha = Xd * math.cos(theta) - Xq * math.sin(theta)
    X_beta = Xd * math.sin(theta) + Xq * math.cos(theta)
    
    # Inverse Clarke transform
    Xa = X_alpha
    Xb = (-X_alpha + math.sqrt(3) * X_beta) / 2
    Xc = (-X_alpha - math.sqrt(3) * X_beta) / 2
    
    # Direct Inverse Park Transform (dq -> abc)
    # Xa = math.cos(theta) * Xd - math.sin(theta) * Xq
    # Xb = math.cos(theta - 2*math.pi/3) * Xd - math.sin(theta - 2*math.pi/3) * Xq
    # Xc = math.cos(theta + 2*math.pi/3) * Xd - math.sin(theta + 2*math.pi/3) * Xq
    
    return Xa, Xb, Xc
