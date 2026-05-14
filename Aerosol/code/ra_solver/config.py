# Layer height sweep
T_MIN = 0.1        # mm
T_MAX = 0.4        # mm
N_POINTS = 500     # number of interpolation points for the continuous curve

# Discrete points to mark on the plot
T_MARKS = [0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5]  # mm

# Error coefficient fractions (derived from Table 1, theta=90 deg, t=0.25 mm reference)
# eps_x = ALPHA_X * t,  eps_y = ALPHA_Y * t
# ALPHA_X = 0.0660
# ALPHA_Y = 0.2232
ALPHA_X = 0
ALPHA_Y = 0

# Stratification angle (fixed for this solver)
THETA_DEG = 90.0

# Numerical integration fallback (cross-check only, not used in sweep)
N_QUAD = 10000

# Profile landscape plot
N_LAYERS = 6   # number of layer periods to tile in the profile view

