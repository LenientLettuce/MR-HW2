"""
Part (b): uncertainty in the wall angle alpha from two noisy LiDAR ranges.

Model:  alpha = atan2(a*cos(theta) - b, a*sin(theta))
Linearized (alpha = 0):  sigma_alpha = (sigma_r / D) * cot(theta) * sqrt(1 + cos(theta)^2)

Claude was used for writing this script from the derived equations 
"""
import numpy as np

# ---- settings----
sigma_r = 0.01                       # range noise std dev [m]
D = 0.8                              # perpendicular distance to wall [m]
angles_deg = [15, 30, 45, 60, 75]    # theta values to evaluate [deg]
# --------------------------------


def sigma_alpha_formula(theta, D, sigma_r):
    """First-order (Jacobian) result, in radians."""
    return (sigma_r / D) / np.tan(theta) * np.sqrt(1 + np.cos(theta) ** 2)


print(f"sigma_r = {sigma_r} m, D = {D} m\n")
header = f"{'theta':>6} | {'formula [deg]':>13}"
print(header)
print("-" * len(header))

for th_deg in angles_deg:
    th = np.radians(th_deg)
    f_deg = np.degrees(sigma_alpha_formula(th, D, sigma_r))
    print(f"{th_deg:>5}° | {f_deg:>13.3f}")