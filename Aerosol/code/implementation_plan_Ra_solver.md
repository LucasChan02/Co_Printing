# Implementation Plan: Surface Roughness Ra Solver for Vertical FDM Walls
**Based on:** Li, Haghighi & Yang (2019), *IISE Transactions*, 51(2), 124–135.  
**Case:** Vertical FDM surface, stratification angle θ = 90°, layer height t ∈ [0.1, 0.4] mm.

---

## 1. Mathematical Foundation

### 1.1 Profile geometry at θ = 90°

At θ = 90° the paper states that the surface profile is represented **solely by the parabolic function** — the linear component vanishes (Section 3.1, Figure 1). The profile is therefore a downward-opening parabola repeating once per layer.

> **Terminology note:** The user referred to "hyperbolas." The profile in the paper is *parabolic*, not hyperbolic. The two terms describe different conic sections; the paper explicitly derives and validates a parabolic curve. The implementation must use the parabolic form.

### 1.2 Error coefficients

Two scalar error coefficients account for deviations of the printed layer from the nominal geometry (Equations 2–8 of the paper):

- **εx**: deviation of layer thickness in the X-direction (along the surface)
- **εy**: deviation of layer thickness in the Y-direction (in the build direction)

Both are modelled as normally distributed. For the deterministic Ra calculation the **mean values** (μεx, μεy) from Table 1 of the paper are used. At θ = 90°, t = 0.25 mm (the paper's experimental condition):

| Parameter | Value (mm) |
|-----------|-----------|
| μεx       | 0.0165    |
| μεy       | 0.0558    |
| σεx       | 0.0149    |
| σεy       | 0.0073    |

Because the solver must sweep t across [0.1, 0.4] mm — not only the single experimental point of 0.25 mm — the error coefficients must be scaled. The physically motivated choice is to treat them as **fixed fractions of the nominal layer thickness**, computed from the paper's reference point:

```
alpha_x = mu_ex_ref / t_ref = 0.0165 / 0.25 = 0.0660
alpha_y = mu_ey_ref / t_ref = 0.0558 / 0.25 = 0.2232
```

For any layer height t:
```
eps_x(t) = alpha_x * t
eps_y(t) = alpha_y * t
```

This scaling is the only defensible choice for a parametric sweep; the alternative of using fixed absolute values from t = 0.25 mm would misrepresent the geometry at other layer heights. The implementation plan marks this as a user-adjustable parameter so it can be overridden if the user supplies their own measured error data.

### 1.3 Surface profile function

The parabolic profile function in the XY coordinate system (Equation 2 of the paper) is:

```
A = t - eps_x          # effective layer base length
B = t - 2*eps_y        # effective parabola height parameter

f_p(x) = -2*(B/A**2)*x**2 + 2*(B/A)*x,    x in [0, A]
```

The parabola has:
- zeros at x = 0 and x = A
- a maximum of B/2 at x = A/2

### 1.4 Mean profile line at θ = 90°

From Equation 7 with cot(90°) = 0:

```
f2 = (1/3) * B          # a constant
```

This is the mean deviation of the parabolic profile from the zero baseline, matching Equation 6 evaluated at θ = 90°.

### 1.5 Ra integral

From Equation 8 at θ = 90° (sin 90° = 1):

```
Ra = (1/A) * integral from 0 to A of |f_p(x) - f2| dx
```

### 1.6 Closed-form analytical solution

The integral above has an exact closed form, which the coding agent should implement as the primary path (fast, exact, no quadrature error).

**Derivation outline** (for the agent to verify and implement):

Substitute u = x/A, giving dx = A du and u ∈ [0, 1]:

```
f_p(x) - f2 = B * g(u),    where g(u) = 2u(1-u) - 1/3 = 2u - 2u^2 - 1/3
```

Ra reduces to:

```
Ra = B * integral from 0 to 1 of |g(u)| du
```

Find the zeros of g(u) = 0 (i.e., 2u² - 2u + 1/3 = 0):

```
u1 = (1 - 1/sqrt(3)) / 2
u2 = (1 + 1/sqrt(3)) / 2
```

g(u) > 0 on (u1, u2) and ≤ 0 outside. Because the integral of g(u) over [0,1] equals zero, the positive and negative areas are equal in magnitude, giving:

```
integral of |g(u)| du = 2 * integral from u1 to u2 of g(u) du = 2 / (9*sqrt(3))
```

**Final closed-form result:**

```
Ra_analytical = (t - 2*eps_y) * 2 / (9 * sqrt(3))
```

In terms of the proportional error fractions:

```
Ra_analytical = t * (1 - 2*alpha_y) * 2 / (9 * sqrt(3))
```

This shows that Ra scales **linearly with t** when error fractions are held constant — a physically intuitive result that the agent should verify against the numerical integration.

---

## 2. File and Module Structure

```
ra_solver/
├── main.py           # entry point: runs sweep, calls plot
├── model.py          # all physics/math functions
├── plot.py           # all matplotlib plotting logic
└── config.py         # all user-adjustable constants
```

---

## 3. config.py — All Tunable Parameters

```python
# Layer height sweep
T_MIN = 0.1        # mm
T_MAX = 0.4        # mm
N_POINTS = 500     # number of interpolation points for the continuous curve

# Discrete points to mark on the plot
T_MARKS = [0.1, 0.2, 0.3, 0.4]  # mm

# Error coefficient fractions (derived from Table 1, theta=90 deg, t=0.25 mm)
ALPHA_X = 0.0660   # eps_x = ALPHA_X * t
ALPHA_Y = 0.2232   # eps_y = ALPHA_Y * t

# Stratification angle (fixed for this solver)
THETA_DEG = 90.0

# Numerical integration fallback
N_QUAD = 10000     # number of quadrature points for scipy.integrate.quad cross-check

# Profile landscape plot
N_LAYERS = 5       # number of layer periods tiled in each profile landscape figure
```

---

## 4. model.py — Physics Functions

The agent should implement the following functions in this module.

### 4.1 Error coefficients

```python
def error_coefficients(t, alpha_x, alpha_y):
    """
    Return (eps_x, eps_y) for a given layer height t.
    Both are the mean values scaled proportionally from Table 1.
    """
    eps_x = alpha_x * t
    eps_y = alpha_y * t
    return eps_x, eps_y
```

### 4.2 Profile parameters

```python
def profile_params(t, eps_x, eps_y):
    """
    Return effective base length A and height parameter B.
    A = t - eps_x
    B = t - 2*eps_y
    Raises ValueError if A <= 0 or B <= 0.
    """
```

### 4.3 Parabolic profile (vectorised)

```python
def f_parabola(x, A, B):
    """
    Evaluate f_p(x) = -2*(B/A^2)*x^2 + 2*(B/A)*x
    x may be a numpy array; returns array of same shape.
    """
```

### 4.4 Mean profile line

```python
def f_mean(B):
    """
    Return the constant mean line value f2 = B/3.
    Valid at theta = 90 deg only.
    """
    return B / 3.0
```

### 4.5 Analytical Ra

```python
import numpy as np

def Ra_analytical(t, alpha_x=ALPHA_X, alpha_y=ALPHA_Y):
    """
    Closed-form Ra for a vertical FDM surface (theta=90 deg).
    
    Ra = (t - 2*eps_y) * 2 / (9 * sqrt(3))
    
    Parameters
    ----------
    t : float or numpy array
        Layer height in mm.
    alpha_x, alpha_y : float
        Proportional error fractions.

    Returns
    -------
    Ra : same shape as t, in mm.
    """
    eps_x = alpha_x * t   # used for A; note A doesn't appear in final expression
    eps_y = alpha_y * t
    B = t - 2.0 * eps_y
    Ra = B * 2.0 / (9.0 * np.sqrt(3.0))
    return Ra
```

### 4.6 Numerical Ra (cross-check only)

```python
from scipy import integrate

def Ra_numerical(t, alpha_x=ALPHA_X, alpha_y=ALPHA_Y, n_quad=N_QUAD):
    """
    Compute Ra by direct numerical integration using scipy.integrate.quad.
    Used only to validate Ra_analytical. Not used in the final plot loop.
    
    Procedure:
      1. Compute eps_x, eps_y.
      2. Compute A, B.
      3. Define integrand = |f_parabola(x, A, B) - f_mean(B)|.
      4. Integrate over [0, A] and divide by A.
    """
```

### 4.7 Validation check

```python
def validate_analytical_vs_numerical(t_values, tol=1e-6):
    """
    For each t in t_values, compute both Ra_analytical and Ra_numerical.
    Assert that |Ra_analytical - Ra_numerical| / Ra_analytical < tol for each.
    Print a summary table. Raise AssertionError if any check fails.
    """
```

---

## 5. plot.py — Plotting Logic

Two functions are implemented in this module.

### 5.1 Ra sweep plot

```python
def plot_Ra_vs_t(t_array, Ra_array, t_marks, Ra_marks, output_path="Ra_vs_t.png"):
    """
    Parameters
    ----------
    t_array  : 1-D numpy array of t values in mm (the continuous sweep)
    Ra_array : 1-D numpy array of Ra values in mm (same length)
    t_marks  : list of t values to mark with distinctive points [0.1, 0.2, 0.3, 0.4]
    Ra_marks : list of corresponding Ra values
    output_path : path for the saved PNG figure
    """
```

**Plot requirements:**

- Figure size: 8 × 5 inches, 150 dpi minimum for the saved file.
- X-axis: layer height t in mm, range [0.08, 0.42] for visual padding.
- Y-axis: Ra in µm (convert from mm by multiplying by 1000). Label: "Ra (µm)".
- Continuous curve: solid blue line, linewidth 1.8, label "Analytical model (θ = 90°)".
- Four discrete marks (t = 0.1, 0.2, 0.3, 0.4 mm): red filled circles, markersize 8, zorder above the line.
- Each discrete mark annotated with a text label showing the exact Ra value in µm, rounded to two decimal places, offset slightly above-right of the marker to avoid overlap.
- Grid: light grey, linestyle '--', alpha 0.5, on both axes.
- Title: "Surface Roughness Ra vs. Layer Height — Vertical FDM Wall (θ = 90°)"
- Legend: lower-right corner.
- The four marked points should also appear in the legend as a single entry "Marked values (t = 0.1, 0.2, 0.3, 0.4 mm)".
- Save as PNG to output_path using `plt.savefig(..., bbox_inches='tight')`.
- Also call `plt.show()` after saving.

**Unit conversion note:** Ra_analytical returns values in mm (consistent with the paper's unit system). The plot displays µm for readability. The conversion factor 1 mm = 1000 µm must be applied inside plot.py, not in model.py.

### 5.2 Profile landscape plots

```python
def plot_profile_landscape(t_marks, alpha_x=ALPHA_X, alpha_y=ALPHA_Y,
                           n_layers=N_LAYERS, n_pts_per_layer=300,
                           output_dir="."):
```

Produces **one PNG file per entry in t_marks**, saved to `output_dir` with the naming pattern `profile_landscape_t{t:.2f}mm.png` (e.g. `profile_landscape_t0.10mm.png`). Each file is a standalone figure — not a multi-panel grid — so cases can be placed side-by-side for comparison.

**Fixed shared axis limits (for direct cross-case comparison):**

Both x and y limits are derived once from the geometry of `max(t_marks)` (the largest layer height, which has the widest and tallest profile). A 6 % x-margin and 15 % y-margin are added so profiles are never clipped.

```
x_data_max = N_LAYERS * A_max * 1000   [µm]
y_data_max = (B_max / 2) * 1000        [µm, peak height]
xlim = (-0.06 * x_data_max,  1.06 * x_data_max)
ylim = (-0.15 * y_data_max,  1.15 * y_data_max)
```

**Per-figure contents:**

- Figure size: 9 × 4 inches, 150 dpi.
- **Navy solid curve** (lw 1.5): tiled parabolic profile over `N_LAYERS` periods.
- **Red dashed horizontal line**: mean line at f₂ = B/3 (µm), labelled with its value.
- **Green shading** (alpha 0.35): regions where profile > mean (positive Ra contribution).
- **Orange shading** (alpha 0.35): regions where profile < mean (negative Ra contribution; the shoulders at period edges where the parabola approaches zero).
- **Grey dashed verticals** (lw 0.8, alpha 0.6): period boundaries at x = k·A·1000 µm for k = 1 … N_LAYERS−1.
- Light grey background grid, linestyle '--', alpha 0.4.
- Title: includes t, Ra, A, and B values so the figure is self-contained.
- Legend: upper right, fontsize 8.

**Geometry note:** The parabola peak is B/2, and the mean line sits at B/3 (one-third of full height). The crossover between positive and negative regions occurs at u₁ = (1 − 1/√3)/2 ≈ 0.211 and u₂ = (1 + 1/√3)/2 ≈ 0.789 of each period length A. The equal areas of green and orange shading visually confirm that the Ra integral is symmetric about the mean line.

---

## 6. main.py — Entry Point

```python
import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
from config import T_MIN, T_MAX, N_POINTS, T_MARKS
from model import Ra_analytical, validate_analytical_vs_numerical
from plot import plot_Ra_vs_t, plot_profile_landscape

if __name__ == "__main__":
    # 1. Validate analytical formula against numerical integration
    print("=== Validation: analytical vs. numerical ===")
    validate_analytical_vs_numerical(T_MARKS)

    # 2. Compute continuous sweep
    t_array = np.linspace(T_MIN, T_MAX, N_POINTS)
    Ra_array = Ra_analytical(t_array)

    # 3. Compute discrete marked points
    t_marks = np.array(T_MARKS)
    Ra_marks = Ra_analytical(t_marks)

    # 4. Print summary table for the four marked values
    print("\n=== Ra at marked layer heights ===")
    print(f"{'t (mm)':>10}  {'Ra (mm)':>12}  {'Ra (µm)':>12}")
    for t_val, Ra_val in zip(t_marks, Ra_marks):
        print(f"{t_val:>10.3f}  {Ra_val:>12.6f}  {Ra_val*1000:>12.4f}")

    # 5. Plot Ra vs t sweep
    plot_Ra_vs_t(t_array, Ra_array, list(t_marks), list(Ra_marks))

    # 6. Profile landscape (individual files, shared fixed scale)
    print("\n=== Generating profile landscape plots ===")
    script_dir = os.path.dirname(__file__)
    plot_profile_landscape(list(t_marks), output_dir=script_dir)
```

---

## 7. Dependencies

All standard; no exotic packages required.

```
numpy
scipy
matplotlib
```

Install via: `pip install numpy scipy matplotlib`

---

## 8. Expected Numerical Output

With the proportional error model (alpha_x = 0.066, alpha_y = 0.2232):

```
Ra_analytical = t * (1 - 2*0.2232) * 2 / (9*sqrt(3))
              = t * 0.5536 * 0.12830
              = t * 0.07102   [mm, with t in mm]
              = t * 71.02     [µm, with t in mm]
```

Expected marked values:

| t (mm) | Ra (µm) |
|--------|---------|
| 0.1    | 7.10    |
| 0.2    | 14.20   |
| 0.3    | 21.31   |
| 0.4    | 28.41   |

The linear relationship is a direct consequence of scaling error fractions proportionally with t. If the user overrides with fixed absolute error values (e.g., from a single measurement campaign at one specific t), the relationship will become nonlinear. The config.py flags make this straightforward to test.

---

## 9. Key Implementation Cautions

1. **Avoid division by zero.** If alpha_x ≥ 1 or alpha_y ≥ 0.5, A or B will go negative. Add a guard in `profile_params()` and raise a descriptive `ValueError`.

2. **Unit consistency.** The paper uses millimetres throughout. Keep all computations in mm; convert to µm only at the plot boundary.

3. **The analytical formula is exact.** The numerical integration is for validation only. Do not use `scipy.integrate.quad` inside the sweep loop; it is 100–1000× slower than the vectorised `Ra_analytical`.

4. **Fourier series (θ = 90° hybrid case) is not needed.** Equations 13–24 in the paper apply to the hybrid additive-subtractive case. This solver covers the pure AM case at θ = 90°, for which the closed-form in Section 3.3 (Equations 2–8) is sufficient and complete.

5. **Profile is parabolic, not hyperbolic.** The paper derives the profile using a parabola (second-degree polynomial). Do not substitute a hyperbola; it would contradict the model's derivation and the experimental validation in Table 3.
