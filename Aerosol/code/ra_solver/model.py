import numpy as np
from scipy import integrate

from config import ALPHA_X, ALPHA_Y, N_QUAD


def error_coefficients(t, alpha_x=ALPHA_X, alpha_y=ALPHA_Y):
    """Return (eps_x, eps_y) scaled proportionally from Table 1 reference point."""
    return alpha_x * t, alpha_y * t


def profile_params(t, eps_x, eps_y):
    """
    Return effective base length A and height parameter B.
      A = t - eps_x
      B = t - 2*eps_y
    Raises ValueError if either goes non-positive (unphysical geometry).
    """
    A = t - eps_x
    B = t - 2.0 * eps_y
    if np.any(A <= 0):
        raise ValueError(
            f"A = t - eps_x = {A} <= 0. Check ALPHA_X (must be < 1)."
        )
    if np.any(B <= 0):
        raise ValueError(
            f"B = t - 2*eps_y = {B} <= 0. Check ALPHA_Y (must be < 0.5)."
        )
    return A, B


def f_parabola(x, A, B):
    """
    Parabolic profile: f_p(x) = -2*(B/A^2)*x^2 + 2*(B/A)*x, x in [0, A].
    x may be a numpy array.
    """
    return -2.0 * (B / A**2) * x**2 + 2.0 * (B / A) * x


def f_mean(B):
    """Mean profile line constant at theta = 90 deg: f2 = B/3."""
    return B / 3.0


def Ra_analytical(t, alpha_x=ALPHA_X, alpha_y=ALPHA_Y):
    """
    Closed-form Ra for a vertical FDM surface (theta = 90 deg).

    Derived by substituting u = x/A and integrating |g(u)| = |2u(1-u) - 1/3|
    over [0, 1], which evaluates to 2/(9*sqrt(3)).

    Ra = (t - 2*eps_y) * 2 / (9 * sqrt(3))
       = B * 2 / (9 * sqrt(3))

    Parameters
    ----------
    t : float or numpy array
        Layer height in mm.

    Returns
    -------
    Ra in mm (same shape as t).
    """
    eps_y = alpha_y * t
    B = t - 2.0 * eps_y
    return B * 2.0 / (9.0 * np.sqrt(3.0))


def Rz_analytical(t, alpha_x=ALPHA_X, alpha_y=ALPHA_Y):
    """
    Rz for a vertical FDM surface (theta = 90 deg).

    Sampling length = one layer period (length A).
    Within each period: peak = B/2 at x = A/2, valley = 0 at x = 0 and x = A.
    Peak-to-valley per period = B/2. All periods are identical, so the average
    Rz over the assessment length equals the single-period value.

    Rz = B / 2 = (t - 2*eps_y) / 2

    Parameters
    ----------
    t : float or numpy array
        Layer height in mm.

    Returns
    -------
    Rz in mm (same shape as t).
    """
    eps_y = alpha_y * t
    B = t - 2.0 * eps_y
    return B / 2.0


def Ra_numerical(t, alpha_x=ALPHA_X, alpha_y=ALPHA_Y, n_quad=N_QUAD):
    """
    Compute Ra by direct numerical integration (scalar t only).
    Used exclusively to validate Ra_analytical — not called in the sweep loop.
    """
    t = float(t)
    eps_x, eps_y = error_coefficients(t, alpha_x, alpha_y)
    A, B = profile_params(t, eps_x, eps_y)
    f2 = f_mean(B)

    integrand = lambda x: abs(f_parabola(x, A, B) - f2)  # noqa: E731
    result, _ = integrate.quad(integrand, 0.0, A, limit=n_quad)
    return result / A


def validate_analytical_vs_numerical(t_values, tol=1e-6):
    """
    For each t in t_values compare Ra_analytical to Ra_numerical.
    Prints a summary table and raises AssertionError if any relative error
    exceeds tol.
    """
    print(f"\n{'t (mm)':>10}  {'Ra_analyt (mm)':>16}  {'Ra_numer (mm)':>15}  {'rel err':>10}")
    print("-" * 60)
    for t in t_values:
        ra_a = Ra_analytical(float(t))
        ra_n = Ra_numerical(float(t))
        rel_err = abs(ra_a - ra_n) / ra_a
        print(f"{t:>10.3f}  {ra_a:>16.8f}  {ra_n:>15.8f}  {rel_err:>10.2e}")
        assert rel_err < tol, (
            f"Validation failed at t={t}: rel_err={rel_err:.2e} >= tol={tol:.2e}"
        )
    print("All validation checks passed.\n")
