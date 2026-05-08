import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline

# --- 1. Data from the user ---
# Categories are the angles in degrees
angles_deg = np.array([0, 15, 30, 45, 60, 75, 90])

# Minimum and maximum distance values
d_min = np.array([473, 450, 404, 320, 245, 250, 250])
d_max = np.array([1180, 1168, 872, 1072, 1005, 930, 852])

# Convert angles from degrees to radians for plotting
angles_rad = np.deg2rad(angles_deg)

# --- 2. Create smooth curves for a more polished look ---
# To create a smooth curve, we need more points than the original data.
# We'll generate 300 points for a smooth transition between the original angles.
angles_rad_smooth = np.linspace(angles_rad.min(), angles_rad.max(), 300)

# Create spline functions for interpolation
spline_min = make_interp_spline(angles_rad, d_min)
spline_max = make_interp_spline(angles_rad, d_max)

# Calculate the smoothed distance values
d_min_smooth = spline_min(angles_rad_smooth)
d_max_smooth = spline_max(angles_rad_smooth)

# --- 3. Plotting the radar chart ---
# Initialize a figure and a polar subplot
fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))

# Plot the smooth line for the maximum distance
ax.plot(angles_rad_smooth, d_max_smooth, label='Max Reachable Distance', color='cornflowerblue', linewidth=2)

# Plot the smooth line for the minimum distance
ax.plot(angles_rad_smooth, d_min_smooth, label='Min Reachable Distance', color='lightcoral', linewidth=2)

# Fill the area between the min and max lines to show the viable range
# Using a light shadow effect with the 'alpha' parameter for transparency
ax.fill_between(angles_rad_smooth, d_min_smooth, d_max_smooth, color='lightsteelblue', alpha=0.4, label='Viable Range')

# --- 4. Customizing the chart's appearance ---
# Set the direction for the angles to be anti-clockwise
ax.set_theta_direction(-1)

# Set the 0-degree angle to be at the top (12 o'clock)
ax.set_theta_offset(np.pi / 2.0)

# Set the plot to be a quarter circle (0 to 90 degrees)
ax.set_thetalim(0, np.pi / 2.0)

# Set the labels for the angles
ax.set_xticks(np.deg2rad(np.arange(0, 105, 15)))
ax.set_xticklabels(['0°', '15°', '30°', '45°', '60°', '75°', '90°'])

# Set the radial axis (distance) limits. We add a little padding.
ax.set_ylim(0, d_max.max() + 100)

# Add a title to the chart
ax.set_title('Reachable Range Analysis', va='bottom', fontsize=16, fontweight='bold')

# Add a legend to identify the lines and the filled area
ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))

# Use a tight layout to prevent labels from overlapping
plt.tight_layout()

# --- 5. Display the chart ---
plt.show()