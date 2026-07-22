"""Planted magnetic dipole helpers for simulation."""

from __future__ import annotations

import math


def dipole_anomaly_nt(x, y, target_x, target_y, strength_nt, soft_m=1.0):
    """
    Scalar anomaly A / (r^3 + soft^3), matching the offline niot magnetic sim.

    strength_nt is the near-field amplitude constant in the same units as the
    rest of the inflated Gazebo mag pipeline (nT after Tesla conversion).
    """
    dx = float(x) - float(target_x)
    dy = float(y) - float(target_y)
    r = math.hypot(dx, dy)
    soft = max(float(soft_m), 1e-3)
    return float(strength_nt) / (r * r * r + soft * soft * soft)


def apply_vertical_dipole(bx, by, bz, anomaly_nt):
    """Add a vertical (Bz) dipole contribution and return updated axes."""
    return float(bx), float(by), float(bz) + float(anomaly_nt)
