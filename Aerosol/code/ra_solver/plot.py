import os

import matplotlib
matplotlib.use('Agg')   # non-interactive backend: no popup windows
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from config import ALPHA_X, ALPHA_Y, N_LAYERS
from model import error_coefficients, profile_params, f_parabola, f_mean, Ra_analytical, Rz_analytical


def plot_Ra_vs_t(t_array, Ra_array, Rz_array,
                 t_marks, Ra_marks, Rz_marks,
                 output_path="Ra_Rz_vs_t.png"):
    """
    Plot Ra and Rz vs layer height on the same axes.

    Parameters
    ----------
    t_array            : 1-D numpy array of t values in mm (continuous sweep)
    Ra_array, Rz_array : 1-D numpy arrays in mm (same length as t_array)
    t_marks            : list of discrete t values to mark
    Ra_marks, Rz_marks : corresponding Ra/Rz values in mm
    output_path        : path for the saved PNG figure
    """
    MM_TO_UM = 1000.0

    Ra_um      = np.asarray(Ra_array) * MM_TO_UM
    Rz_um      = np.asarray(Rz_array) * MM_TO_UM
    Ra_marks_um = np.asarray(Ra_marks) * MM_TO_UM
    Rz_marks_um = np.asarray(Rz_marks) * MM_TO_UM
    t_marks     = np.asarray(t_marks)

    fig, ax = plt.subplots(figsize=(8, 5))

    ax.plot(t_array, Ra_um, color="steelblue",  linewidth=1.0)
    ax.plot(t_array, Rz_um, color="darkorange", linewidth=1.0)

    ax.scatter(t_marks, Ra_marks_um, color="steelblue",  marker="o", s=20, zorder=5)
    ax.scatter(t_marks, Rz_marks_um, color="darkorange", marker="^", s=20, zorder=5)

    handles = [
        Line2D([0], [0], color='steelblue',  marker='o', linewidth=1.0, markersize=4, label='Ra'),
        Line2D([0], [0], color='darkorange', marker='^', linewidth=1.0, markersize=4, label='Rz'),
    ]
    t_lo = min(float(t_array[0]),  float(t_marks[0]))
    t_hi = max(float(t_array[-1]), float(t_marks[-1]))
    ax.set_xlim(t_lo - 0.02, t_hi + 0.02)
    ax.set_xlabel("Layer height t (mm)")
    ax.set_ylabel("Roughness (µm)")
    ax.set_title("Surface Roughness Ra & Rz vs. Layer Height")
    ax.grid(color='grey', linestyle='--', alpha=0.6)
    ax.legend(handles=handles, loc="upper left")

    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_profile_landscape(t_marks, alpha_x=ALPHA_X, alpha_y=ALPHA_Y,
                           n_layers=5, n_pts_per_layer=300,
                           output_dir="."):
    """
    Save one PNG per entry in t_marks to output_dir.

    Filename pattern: profile_landscape_t{t:.2f}mm.png

    Y-axis is recentred so the mean line sits at y = 0. Profile values above
    zero are the positive Ra contribution (green); below zero are negative
    (orange). All cases share fixed axis limits derived from the largest t.
    """
    # ── annotation text positions (edit directly to reposition labels) ──────
    VALLEY_TEXT_X_OFFSET = 50   # µm right of valley marker line
    VALLEY_TEXT_Y_OFFSET = -20    # µm shift from segment midpoint (+ = up)
    PEAK_TEXT_X_OFFSET   = 28   # µm right of peak marker line
    PEAK_TEXT_Y_OFFSET   = 40    # µm shift from segment midpoint (+ = up)
    # ── y-axis display window (µm) — change to zoom in/out ───────────────────
    Y_PLOT_MIN = -100           # µm  bottom edge
    Y_PLOT_MAX =  100           # µm  top edge
    # ─────────────────────────────────────────────────────────────────────────

    t_marks = list(t_marks)

    # Shared xlim derived from the largest t so all plots use the same zoom level
    t_max = max(t_marks)
    eps_x_max, eps_y_max = error_coefficients(t_max, alpha_x, alpha_y)
    A_max, _ = profile_params(t_max, eps_x_max, eps_y_max)
    x_right_max = n_layers * A_max * 1000
    x_pad_shared = 0.06 * x_right_max
    shared_xlim = (-x_pad_shared, x_right_max + x_pad_shared)

    saved = []
    for t in t_marks:
        eps_x, eps_y = error_coefficients(t, alpha_x, alpha_y)
        A, B = profile_params(t, eps_x, eps_y)
        f2  = f_mean(B)
        Ra  = Ra_analytical(t, alpha_x, alpha_y)
        Rz  = Rz_analytical(t, alpha_x, alpha_y)

        x_segs, y_segs = [], []
        for k in range(12):
            endpoint = (k == n_layers - 1)
            x_local = np.linspace(0.0, A, n_pts_per_layer, endpoint=endpoint)
            x_segs.append(x_local + k * A)
            y_segs.append(f_parabola(x_local, A, B))

        x         = np.concatenate(x_segs) * 1000          # mm → µm
        y_raw     = np.concatenate(y_segs)                  # mm, absolute
        y         = (y_raw - f2) * 1000                     # µm, mean at 0
        valley_um = -f2 * 1000                              # valley y (negative)
        peak_um   = (B / 2.0 - f2) * 1000                  # peak y (positive)

        fig, ax = plt.subplots(figsize=(18, 6))

        ax.plot(x, y, color="navy", linewidth=0.8, label="Surface profile")
        ax.axhline(0, color="red", linestyle="--", linewidth=0.6,
                   label=f"Mean line f₂ = B/3 = {f2 * 1000:.2f} µm (y = 0)")

        ax.fill_between(x, y, 0, where=(y >= 0), color="green",  alpha=0.15)
        ax.fill_between(x, y, 0, where=(y <= 0), color="orange", alpha=0.15)

        # Marker 1: valley bottom → mean line  (at x = A, second valley)
        x_v2 = A * 1000
        ax.annotate(
            '',
            xy=(x_v2, valley_um),
            xytext=(x_v2, 0.0),
            arrowprops=dict(arrowstyle='-', color='darkviolet', linewidth=0.4),
        )
        ax.text(
            x_v2  + VALLEY_TEXT_X_OFFSET,
            valley_um / 2 + VALLEY_TEXT_Y_OFFSET,
            f"Δ = {-valley_um:.2f} µm",
            color='darkviolet', fontsize=7.5, va='center',
        )

        # Marker 2: mean line → first peak  (at x = A/2)
        x_peak = (A / 2.0) * 1000
        ax.annotate(
            '',
            xy=(x_peak, 0.0),
            xytext=(x_peak, peak_um),
            arrowprops=dict(arrowstyle='-', color='darkgreen', linewidth=0.4),
        )
        ax.text(
            x_peak + PEAK_TEXT_X_OFFSET,
            peak_um / 2 + PEAK_TEXT_Y_OFFSET,
            f"Δ = {peak_um:.2f} µm",
            color='darkgreen', fontsize=7.5, va='center',
        )

        xlim = shared_xlim
        ylim = (Y_PLOT_MIN, Y_PLOT_MAX)

        ax.set_xlim(xlim)
        ax.set_ylim(ylim)
        ax.set_yticks([-50, 0, 50])
        ax.set_aspect('equal')
        ax.set_title(
            f"t = {t:.2f} mm  |  Ra = {Ra * 1000:.2f} µm  |  Rz = {Rz * 1000:.2f} µm  |  "
            f"A = {A * 1000:.1f} µm  |  B = {B * 1000:.1f} µm"
        )
        ax.set_xlabel("Position along surface (µm)")
        ax.set_ylabel("Deviation (µm)")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(color='grey', linestyle='--', alpha=0.6)

        fname = f"profile_landscape_t{t:.2f}mm.png"
        out   = os.path.join(output_dir, fname)
        fig.savefig(out, dpi=150, bbox_inches="tight")
        plt.close(fig)
        saved.append(out)

    print("Saved profile landscape files:")
    for p in saved:
        print(f"  {p}")
