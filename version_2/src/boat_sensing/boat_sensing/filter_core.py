"""ROS-independent magnetometer filter math."""

import math
from collections import deque


TESLA_TO_NT = 1.0e9


def vector_scalar(bx, by, bz):
    return math.sqrt(bx * bx + by * by + bz * bz)


def tesla_to_nt(value_t):
    return value_t * TESLA_TO_NT


class MovingAverageLowPass:
    """Simple moving-average low-pass filter."""

    def __init__(self, window_size=5):
        self.window_size = max(1, int(window_size))
        self._samples = deque(maxlen=self.window_size)

    def reset(self):
        self._samples.clear()

    def update(self, value):
        self._samples.append(float(value))
        return sum(self._samples) / len(self._samples)


class SpikeRejectFilter:
    """Reject samples that jump more than n_sigma from recent mean."""

    def __init__(self, history_size=20, n_sigma=3.0, min_std_nt=1.0):
        self.history_size = max(2, int(history_size))
        self.n_sigma = float(n_sigma)
        self.min_std_nt = float(min_std_nt)
        self._history = deque(maxlen=self.history_size)
        self.rejected_count = 0
        self.accepted_count = 0
        self._consecutive_rejects = 0

    def reset(self):
        self._history.clear()
        self.rejected_count = 0
        self.accepted_count = 0
        self._consecutive_rejects = 0

    def accept(self, value):
        value = float(value)
        if len(self._history) < 3:
            self._history.append(value)
            self.accepted_count += 1
            self._consecutive_rejects = 0
            return True, value

        mean = sum(self._history) / len(self._history)
        variance = sum((sample - mean) ** 2 for sample in self._history) / len(
            self._history
        )
        std = max(math.sqrt(variance), self.min_std_nt)
        if abs(value - mean) > self.n_sigma * std:
            self.rejected_count += 1
            self._consecutive_rejects += 1
            # Adapt after a sustained step (e.g. teleported into a strong dipole).
            if self._consecutive_rejects >= 5:
                self._history.append(value)
                self._consecutive_rejects = 0
                self.accepted_count += 1
                return True, value
            return False, self._history[-1]

        self._history.append(value)
        self.accepted_count += 1
        self._consecutive_rejects = 0
        return True, value


class MagnetometerFilterChain:
    """Spike reject then low-pass on each magnetometer axis and scalar."""

    def __init__(
        self,
        lowpass_window=5,
        spike_history=20,
        spike_n_sigma=3.0,
        min_std_nt=1.0,
    ):
        self.axes = ('bx', 'by', 'bz', 'scalar')
        self.spike = {
            axis: SpikeRejectFilter(spike_history, spike_n_sigma, min_std_nt)
            for axis in self.axes
        }
        self.lowpass = {
            axis: MovingAverageLowPass(lowpass_window) for axis in self.axes
        }
        self.last_rejected = False

    def reset(self):
        for axis in self.axes:
            self.spike[axis].reset()
            self.lowpass[axis].reset()
        self.last_rejected = False

    def update(self, bx, by, bz):
        scalar = vector_scalar(bx, by, bz)
        raw = {'bx': bx, 'by': by, 'bz': bz, 'scalar': scalar}
        filtered = {}
        rejected_any = False

        for axis in self.axes:
            accepted, value = self.spike[axis].accept(raw[axis])
            if not accepted:
                rejected_any = True
            filtered[axis] = self.lowpass[axis].update(value)

        self.last_rejected = rejected_any
        return filtered

    @property
    def rejected_count(self):
        return sum(filter_.rejected_count for filter_ in self.spike.values())
