"""Mutual-information (information-surfing) planner for TARGET_SEARCH.

Ported from the offline Phase 5 reference and niot mission_manager/info_gain.py
into pure Python so it matches boat_mapping.bayes_core without a numpy dependency.
"""

from __future__ import annotations

import math

from boat_mapping.bayes_core import detection_probability


def belief_entropy(belief):
    """Shannon entropy of a flat probability mass vector."""
    entropy = 0.0
    for value in belief:
        if value > 1e-12:
            entropy -= value * math.log(value)
    return entropy


def _cell_centers(origin_x, origin_y, cell_size, width, height):
    centers = []
    for j in range(height):
        for i in range(width):
            centers.append(
                (
                    origin_x + (i + 0.5) * cell_size,
                    origin_y + (j + 0.5) * cell_size,
                )
            )
    return centers


def detection_surface(
    asv_pos,
    centers,
    p_bg=0.05,
    p_max=0.95,
    d_half=30.0,
):
    """P(detect | target in cell) for every cell given ASV pose."""
    ax, ay = asv_pos
    return [
        detection_probability(
            math.hypot(cx - ax, cy - ay),
            p_bg=p_bg,
            p_max=p_max,
            d_half=d_half,
        )
        for cx, cy in centers
    ]


def posterior_after_reading(belief, p_detect, is_hit):
    posterior = []
    total = 0.0
    for b, p_det in zip(belief, p_detect):
        value = b * p_det if is_hit else b * (1.0 - p_det)
        posterior.append(value)
        total += value
    if total <= 0.0:
        n = len(belief)
        return [1.0 / n] * n
    return [value / total for value in posterior]


def expected_posterior_entropy(
    belief,
    asv_pos,
    centers,
    p_bg=0.05,
    p_max=0.95,
    d_half=30.0,
):
    p_detect = detection_surface(
        asv_pos, centers, p_bg=p_bg, p_max=p_max, d_half=d_half
    )
    p_hit = sum(b * p for b, p in zip(belief, p_detect))
    p_hit = min(max(p_hit, 1e-8), 1.0 - 1e-8)

    post_hit = posterior_after_reading(belief, p_detect, True)
    post_miss = posterior_after_reading(belief, p_detect, False)
    return (
        p_hit * belief_entropy(post_hit)
        + (1.0 - p_hit) * belief_entropy(post_miss)
    )


def mutual_information_gain(
    belief,
    asv_pos,
    centers,
    p_bg=0.05,
    p_max=0.95,
    d_half=30.0,
):
    """Expected entropy reduction from one HIT/MISS observation at asv_pos."""
    return belief_entropy(belief) - expected_posterior_entropy(
        belief, asv_pos, centers, p_bg=p_bg, p_max=p_max, d_half=d_half
    )


class InfoGainPlanner:
    """Score ring candidates by mutual information against a BeliefGrid."""

    def __init__(
        self,
        p_bg=0.05,
        p_max=0.95,
        d_half=30.0,
        radii=(10.0, 20.0, 30.0),
        num_angles=16,
    ):
        self.p_bg = float(p_bg)
        self.p_max = float(p_max)
        self.d_half = float(d_half)
        self.radii = [float(r) for r in radii]
        self.num_angles = int(num_angles)

        self.belief = None
        self.centers = None
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.cell_size = 20.0
        self.width = 0
        self.height = 0
        self.last_gain = 0.0

    def update_belief_grid(
        self,
        data,
        origin_x,
        origin_y,
        resolution,
        width,
        height,
    ):
        width = int(width)
        height = int(height)
        expected = width * height
        if expected <= 0 or len(data) != expected:
            return False
        total = float(sum(data))
        if total <= 0.0:
            return False
        self.belief = [float(v) / total for v in data]
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.cell_size = float(resolution)
        self.width = width
        self.height = height
        self.centers = _cell_centers(
            self.origin_x, self.origin_y, self.cell_size, width, height
        )
        return True

    @property
    def ready(self):
        return self.belief is not None and self.centers is not None

    def plan(self, current_pos, bounds=None, peak_xy=None):
        """
        Return (best_xy, gain).

        Uses true MI when a belief map is available; otherwise falls back to
        peak-seeking among the same ring candidates.
        """
        from boat_mission.path_planning import generate_info_gain_candidates

        candidates = generate_info_gain_candidates(
            current_pos,
            self.radii,
            num_angles=self.num_angles,
            bounds=bounds,
        )
        if not self.ready:
            return self._peak_fallback(current_pos, candidates, peak_xy)

        best = candidates[0]
        best_gain = -1.0
        for candidate in candidates:
            gain = mutual_information_gain(
                self.belief,
                candidate,
                self.centers,
                p_bg=self.p_bg,
                p_max=self.p_max,
                d_half=self.d_half,
            )
            if gain > best_gain:
                best_gain = gain
                best = candidate
        self.last_gain = float(best_gain)
        return best, self.last_gain

    def _peak_fallback(self, current_pos, candidates, peak_xy):
        if peak_xy is None:
            self.last_gain = 0.0
            return candidates[0], 0.0
        px, py = peak_xy
        best = candidates[0]
        best_score = float('inf')
        for candidate in candidates:
            dist_peak = math.hypot(candidate[0] - px, candidate[1] - py)
            dist_move = math.hypot(
                candidate[0] - current_pos[0], candidate[1] - current_pos[1]
            )
            score = dist_peak - 0.15 * dist_move
            if score < best_score:
                best_score = score
                best = candidate
        self.last_gain = 0.0
        return best, 0.0
