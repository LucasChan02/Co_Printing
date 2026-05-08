import sys
import os

# Allow running from any working directory
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np

from config import T_MIN, T_MAX, N_POINTS, T_MARKS
from model import Ra_analytical, Rz_analytical, validate_analytical_vs_numerical
from plot import plot_Ra_vs_t, plot_profile_landscape

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # 1. Validate analytical formula against numerical integration
    print("=== Validation: analytical vs. numerical ===")
    validate_analytical_vs_numerical(T_MARKS)

    # 2. Continuous sweep
    t_array  = np.linspace(T_MIN, max(T_MARKS), N_POINTS)
    Ra_array = Ra_analytical(t_array)
    Rz_array = Rz_analytical(t_array)

    # 3. Discrete marked points
    t_marks = np.array(T_MARKS)
    Ra_marks = Ra_analytical(t_marks)
    Rz_marks = Rz_analytical(t_marks)

    # 4. Summary table
    print("=== Ra and Rz at marked layer heights ===")
    print(f"{'t (mm)':>10}  {'Ra (µm)':>10}  {'Rz (µm)':>10}  {'Rz/Ra':>8}")
    for t_val, Ra_val, Rz_val in zip(t_marks, Ra_marks, Rz_marks):
        print(f"{t_val:>10.3f}  {Ra_val*1000:>10.4f}  {Rz_val*1000:>10.4f}  {Rz_val/Ra_val:>8.4f}")

    # 5. Plot Ra & Rz vs t
    plot_Ra_vs_t(
        t_array, Ra_array, Rz_array,
        list(t_marks), list(Ra_marks), list(Rz_marks),
        output_path=os.path.join(script_dir, "Ra_Rz_vs_t.png"),
    )

    # 6. Profile landscape (individual files, shared fixed scale)
    print("\n=== Generating profile landscape plots ===")
    plot_profile_landscape(list(t_marks), output_dir=script_dir)
