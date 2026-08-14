"""Recursive Bayesian belief map with 1/d³ detection probability."""

from dataclasses import dataclass
import math


def detection_probability(distance, p_bg=0.05, p_max=0.95, d_half=30.0):
    """Soft detection probability inspired by a 1/d³ magnetic response."""
    c_const = d_half ** 3
    k_const = (p_max - p_bg) * c_const
    return p_bg + k_const / (distance ** 3 + c_const)


def classify_observation(anomaly_nt, hit_threshold_nt, miss_threshold_nt):
    """Return HIT, MISS, or ABSTAIN for a continuous anomaly feature."""
    if anomaly_nt >= hit_threshold_nt:
        return 'HIT'
    if anomaly_nt <= miss_threshold_nt:
        return 'MISS'
    return 'ABSTAIN'


@dataclass
class BeliefPeak:
    probability: float
    x: float
    y: float
    cell_x: int
    cell_y: int


@dataclass
class BeliefCentroid:
    """Belief-weighted centre of the high-probability region."""
    x: float
    y: float
    mass: float  # total probability inside the selected region (0..1)
    num_cells: int
    spread_m: float  # belief-weighted RMS radius (positional uncertainty)


class BeliefMap:
    """Uniform-prior grid belief updated by HIT/MISS observations."""

    def __init__(
        self,
        area_size_m=300.0,
        origin_x=-150.0,
        origin_y=-150.0,
        cell_size_m=20.0,
        p_bg=0.05,
        p_max=0.95,
        d_half=30.0,
        hit_threshold_nt=1.5,
        miss_threshold_nt=0.4,
        hit_only=False,
    ):
        self.area_size_m = float(area_size_m)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.cell_size_m = float(cell_size_m)
        self.p_bg = float(p_bg)
        self.p_max = float(p_max)
        self.d_half = float(d_half)
        self.hit_threshold_nt = float(hit_threshold_nt)
        self.miss_threshold_nt = float(miss_threshold_nt)
        self.hit_only = bool(hit_only)

        self.width = max(1, int(math.ceil(self.area_size_m / self.cell_size_m)))
        self.height = self.width
        total = self.width * self.height
        self.belief = [1.0 / total] * total
        self.update_count = 0
        self.hit_count = 0
        self.miss_count = 0
        self.abstain_count = 0

    def cell_center(self, i, j):
        x = self.origin_x + (i + 0.5) * self.cell_size_m
        y = self.origin_y + (j + 0.5) * self.cell_size_m
        return x, y

    def index(self, i, j):
        return j * self.width + i

    def _renormalize(self):
        total = sum(self.belief)
        if total <= 0.0:
            n = len(self.belief)
            self.belief = [1.0 / n] * n
            return
        self.belief = [value / total for value in self.belief]

    def peak(self):
        best_idx = max(range(len(self.belief)), key=lambda idx: self.belief[idx])
        i = best_idx % self.width
        j = best_idx // self.width
        x, y = self.cell_center(i, j)
        return BeliefPeak(
            probability=self.belief[best_idx],
            x=x,
            y=y,
            cell_x=i,
            cell_y=j,
        )

    def weighted_centroid(self, threshold_frac=0.5):
        """Belief-weighted centroid over cells >= threshold_frac * peak belief.

        Selecting the "high-probability region" (cells within a fraction of the
        peak) and averaging their positions weighted by belief yields a
        sub-cell estimate that is smoother and more accurate than the single
        argmax cell. Returns a BeliefCentroid with the region mass and a
        belief-weighted RMS spread as a positional-uncertainty proxy.
        """
        frac = min(max(float(threshold_frac), 0.0), 1.0)
        max_b = max(self.belief) if self.belief else 0.0
        if max_b <= 0.0:
            peak = self.peak()
            return BeliefCentroid(peak.x, peak.y, 0.0, 0, 0.0)

        cutoff = frac * max_b
        sum_w = 0.0
        sum_x = 0.0
        sum_y = 0.0
        selected = []
        for j in range(self.height):
            for i in range(self.width):
                b = self.belief[self.index(i, j)]
                if b >= cutoff:
                    cx, cy = self.cell_center(i, j)
                    sum_w += b
                    sum_x += b * cx
                    sum_y += b * cy
                    selected.append((cx, cy, b))

        if sum_w <= 0.0:
            peak = self.peak()
            return BeliefCentroid(peak.x, peak.y, 0.0, 0, 0.0)

        mean_x = sum_x / sum_w
        mean_y = sum_y / sum_w
        var = 0.0
        for cx, cy, b in selected:
            var += b * ((cx - mean_x) ** 2 + (cy - mean_y) ** 2)
        spread = math.sqrt(var / sum_w)
        return BeliefCentroid(mean_x, mean_y, sum_w, len(selected), spread)

    def update(self, x, y, anomaly_nt, is_calibrated=True):
        """Update belief from one MagAnomaly sample. Returns observation label."""
        if not is_calibrated:
            self.abstain_count += 1
            return 'ABSTAIN'

        label = classify_observation(
            anomaly_nt,
            self.hit_threshold_nt,
            self.miss_threshold_nt,
        )
        if label == 'ABSTAIN':
            self.abstain_count += 1
            return label
        if label == 'MISS' and self.hit_only:
            self.abstain_count += 1
            return 'ABSTAIN'

        for j in range(self.height):
            for i in range(self.width):
                cx, cy = self.cell_center(i, j)
                distance = math.hypot(cx - x, cy - y)
                p_det = detection_probability(
                    distance, self.p_bg, self.p_max, self.d_half
                )
                idx = self.index(i, j)
                if label == 'HIT':
                    self.belief[idx] *= p_det
                else:
                    self.belief[idx] *= (1.0 - p_det)

        self._renormalize()
        self.update_count += 1
        if label == 'HIT':
            self.hit_count += 1
        else:
            self.miss_count += 1
        return label

    def as_row_major(self):
        return list(self.belief)

    def entropy(self):
        entropy = 0.0
        for value in self.belief:
            if value > 1e-12:
                entropy -= value * math.log(value)
        return entropy
