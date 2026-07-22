"""Phase 8 verification helpers (confirmations around a candidate)."""

from __future__ import annotations

import math


def reading_confirms_candidate(
    pose_xy,
    candidate_xy,
    peak_xy,
    peak_p,
    cleaned_anomaly_nt,
    arrival_radius_m=30.0,
    peak_tolerance_m=50.0,
    confirmation_threshold_nt=15.0,
    min_peak_probability=0.30,
):
    """Return True if one MagAnomaly sample counts as a verification hit."""
    if pose_xy is None or candidate_xy is None or peak_xy is None:
        return False
    at_site = (
        math.hypot(pose_xy[0] - candidate_xy[0], pose_xy[1] - candidate_xy[1])
        <= arrival_radius_m
    )
    peak_aligned = (
        math.hypot(peak_xy[0] - candidate_xy[0], peak_xy[1] - candidate_xy[1])
        <= peak_tolerance_m
    )
    strong = float(cleaned_anomaly_nt) >= float(confirmation_threshold_nt)
    confident = float(peak_p) >= float(min_peak_probability)
    return at_site and peak_aligned and strong and confident


class VerificationTracker:
    """Accumulate confirmation readings until the required count is reached."""

    def __init__(
        self,
        confirmations_required=4,
        arrival_radius_m=30.0,
        peak_tolerance_m=50.0,
        confirmation_threshold_nt=15.0,
        min_peak_probability=0.30,
    ):
        self.confirmations_required = int(confirmations_required)
        self.arrival_radius_m = float(arrival_radius_m)
        self.peak_tolerance_m = float(peak_tolerance_m)
        self.confirmation_threshold_nt = float(confirmation_threshold_nt)
        self.min_peak_probability = float(min_peak_probability)
        self.candidate_xy = None
        self.confirmations = 0

    def start(self, candidate_xy):
        self.candidate_xy = (float(candidate_xy[0]), float(candidate_xy[1]))
        self.confirmations = 0

    def reset(self):
        self.candidate_xy = None
        self.confirmations = 0

    @property
    def active(self):
        return self.candidate_xy is not None

    @property
    def complete(self):
        return (
            self.active
            and self.confirmations >= self.confirmations_required
        )

    def register(self, pose_xy, peak_xy, peak_p, cleaned_anomaly_nt):
        if not self.active:
            return False
        if reading_confirms_candidate(
            pose_xy,
            self.candidate_xy,
            peak_xy,
            peak_p,
            cleaned_anomaly_nt,
            arrival_radius_m=self.arrival_radius_m,
            peak_tolerance_m=self.peak_tolerance_m,
            confirmation_threshold_nt=self.confirmation_threshold_nt,
            min_peak_probability=self.min_peak_probability,
        ):
            self.confirmations += 1
        return self.complete
