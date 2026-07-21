"""ROS-independent magnetic calibration and anomaly extraction."""

from collections import deque
from dataclasses import dataclass
import math


def wrap_heading(heading):
    return math.atan2(math.sin(heading), math.cos(heading))


def heading_bin(heading, num_bins):
    """Map heading in [-pi, pi] to an integer bin in [0, num_bins)."""
    if num_bins <= 0:
        return 0
    wrapped = wrap_heading(heading)
    normalized = (wrapped + math.pi) / (2.0 * math.pi)
    return min(num_bins - 1, int(normalized * num_bins))


@dataclass
class AnomalySample:
    raw_nt: float
    baseline_nt: float
    cleaned_anomaly_nt: float
    is_calibrated: bool
    heading_bin: int
    grid_cell_x: int
    grid_cell_y: int
    x: float
    y: float
    heading: float


class BaselineMap:
    """Per-cell (and optional heading-bin) ambient baseline map."""

    def __init__(
        self,
        area_size_m=300.0,
        origin_x=-150.0,
        origin_y=-150.0,
        cell_size_m=20.0,
        num_heading_bins=8,
        min_cell_samples=1,
        reject_residual_nt=5000.0,
    ):
        self.area_size_m = float(area_size_m)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.cell_size_m = float(cell_size_m)
        self.num_heading_bins = int(num_heading_bins)
        self.min_cell_samples = int(min_cell_samples)
        self.reject_residual_nt = float(reject_residual_nt)

        self.grid_size = max(1, int(math.ceil(self.area_size_m / self.cell_size_m)))
        # Keyed by (i, j, heading_bin) -> (mean, count)
        self._bins = {}
        # Cell totals regardless of heading for coverage / neighbor fallback
        self._cell_mean = {}
        self._cell_count = {}

    def cell_indices(self, x, y):
        i = int((x - self.origin_x) / self.cell_size_m)
        j = int((y - self.origin_y) / self.cell_size_m)
        i = max(0, min(self.grid_size - 1, i))
        j = max(0, min(self.grid_size - 1, j))
        return i, j

    def total_cells(self):
        return self.grid_size * self.grid_size

    def cells_sampled(self):
        return len(self._cell_count)

    def coverage_percent(self):
        total = self.total_cells()
        if total <= 0:
            return 0.0
        return 100.0 * self.cells_sampled() / total

    def _neighbor_baseline(self, i, j, preferred_bin):
        values = []
        for di, dj in (
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ):
            ni, nj = i + di, j + dj
            if not (0 <= ni < self.grid_size and 0 <= nj < self.grid_size):
                continue
            key = (ni, nj, preferred_bin)
            if key in self._bins:
                values.append(self._bins[key][0])
            elif (ni, nj) in self._cell_mean:
                values.append(self._cell_mean[(ni, nj)])
        if not values:
            return None
        values.sort()
        return values[len(values) // 2]

    def estimate_baseline(self, x, y, heading):
        i, j = self.cell_indices(x, y)
        hbin = heading_bin(heading, self.num_heading_bins)
        key = (i, j, hbin)
        if key in self._bins:
            return self._bins[key][0], True, i, j, hbin
        if (i, j) in self._cell_mean and self._cell_count[(i, j)] >= self.min_cell_samples:
            return self._cell_mean[(i, j)], True, i, j, hbin
        neighbor = self._neighbor_baseline(i, j, hbin)
        if neighbor is not None:
            return neighbor, True, i, j, hbin
        # Global fallback so newly visited cells (e.g. after teleport) still
        # get an ambient baseline instead of absorbing a local dipole.
        if self._cell_mean:
            global_mean = sum(self._cell_mean.values()) / len(self._cell_mean)
            return global_mean, True, i, j, hbin
        return None, False, i, j, hbin

    def update(self, x, y, heading, raw_nt, allow_update=True):
        if not allow_update:
            return False

        i, j = self.cell_indices(x, y)
        hbin = heading_bin(heading, self.num_heading_bins)
        baseline, calibrated, _, _, _ = self.estimate_baseline(x, y, heading)
        if calibrated and abs(raw_nt - baseline) > self.reject_residual_nt:
            return False

        key = (i, j, hbin)
        if key not in self._bins:
            self._bins[key] = (float(raw_nt), 1)
        else:
            mean, count = self._bins[key]
            count += 1
            mean += (raw_nt - mean) / count
            self._bins[key] = (mean, count)

        if (i, j) not in self._cell_mean:
            self._cell_mean[(i, j)] = float(raw_nt)
            self._cell_count[(i, j)] = 1
        else:
            count = self._cell_count[(i, j)] + 1
            mean = self._cell_mean[(i, j)]
            mean += (raw_nt - mean) / count
            self._cell_mean[(i, j)] = mean
            self._cell_count[(i, j)] = count
        return True


class TemporalHighPass:
    """Along-track temporal high-pass on spatial anomaly features."""

    def __init__(self, window=12, noise_floor_nt=0.0):
        self.window = max(4, int(window))
        self.noise_floor_nt = float(noise_floor_nt)
        self.cleaned_history = deque(maxlen=self.window)
        self.raw_history = deque(maxlen=max(30, self.window * 2))

    def reset(self):
        self.cleaned_history.clear()
        self.raw_history.clear()

    def update(self, spatial_cleaned, raw_nt):
        self.cleaned_history.append(float(spatial_cleaned))
        self.raw_history.append(float(raw_nt))

        track_anomaly = 0.0
        if len(self.raw_history) >= 6:
            sorted_raw = sorted(self.raw_history)
            track_baseline = sorted_raw[max(0, int(0.2 * (len(sorted_raw) - 1)))]
            track_anomaly = max(0.0, raw_nt - track_baseline)

        if len(self.cleaned_history) < 4 or spatial_cleaned <= self.noise_floor_nt:
            return max(spatial_cleaned, track_anomaly)

        slow = sorted(self.cleaned_history)[len(self.cleaned_history) // 2]
        spatial_boost = abs(spatial_cleaned - slow)
        return max(spatial_cleaned, spatial_boost, track_anomaly)


class MagneticCalibrator:
    """Combine baseline map + temporal high-pass into MagAnomaly features."""

    PHASE_CALIBRATING = 'CALIBRATING'
    PHASE_READY = 'READY'
    PHASE_DEGRADED = 'DEGRADED'

    def __init__(
        self,
        baseline_map=None,
        temporal=None,
        ready_coverage_percent=5.0,
        ready_min_cells=8,
        freeze_baseline_when_ready=True,
    ):
        self.baseline = baseline_map or BaselineMap()
        self.temporal = temporal or TemporalHighPass()
        self.ready_coverage_percent = float(ready_coverage_percent)
        self.ready_min_cells = int(ready_min_cells)
        self.freeze_baseline_when_ready = bool(freeze_baseline_when_ready)
        self.phase = self.PHASE_CALIBRATING
        self.samples_processed = 0

    def _refresh_phase(self):
        cells = self.baseline.cells_sampled()
        coverage = self.baseline.coverage_percent()
        if cells >= self.ready_min_cells or coverage >= self.ready_coverage_percent:
            self.phase = self.PHASE_READY
        else:
            self.phase = self.PHASE_CALIBRATING

    def process(self, x, y, heading, raw_nt):
        allow_update = True
        if self.freeze_baseline_when_ready and self.phase == self.PHASE_READY:
            allow_update = False

        self.baseline.update(x, y, heading, raw_nt, allow_update=allow_update)
        baseline, calibrated, i, j, hbin = self.baseline.estimate_baseline(
            x, y, heading
        )
        if baseline is None:
            spatial = 0.0
            baseline_value = float('nan')
            calibrated = False
        else:
            spatial = abs(raw_nt - baseline)
            baseline_value = baseline

        cleaned = self.temporal.update(spatial, raw_nt) if calibrated else 0.0
        self.samples_processed += 1
        self._refresh_phase()

        return AnomalySample(
            raw_nt=float(raw_nt),
            baseline_nt=float(baseline_value) if calibrated else float('nan'),
            cleaned_anomaly_nt=float(cleaned),
            is_calibrated=bool(calibrated),
            heading_bin=int(hbin),
            grid_cell_x=int(i),
            grid_cell_y=int(j),
            x=float(x),
            y=float(y),
            heading=float(heading),
        )
