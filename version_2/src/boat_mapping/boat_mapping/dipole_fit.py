"""Least-squares dipole localization from (x, y, anomaly) samples."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


def model_anomaly_nt(x, y, tx, ty, strength_nt, soft_m):
    """A / (r^3 + soft^3) — same form as the planted dipole."""
    dx = float(x) - float(tx)
    dy = float(y) - float(ty)
    r = math.hypot(dx, dy)
    soft = max(float(soft_m), 1e-3)
    denom = r * r * r + soft * soft * soft
    return float(strength_nt) / denom


@dataclass
class DipoleFix:
    """Result of a dipole least-squares localization."""

    x: float
    y: float
    strength_nt: float
    residual_rms_nt: float
    num_samples: int
    success: bool


class DipoleFitter:
    """Buffer strong anomaly samples and fit a continuous (tx, ty, A)."""

    def __init__(
        self,
        soft_m=20.0,
        min_anomaly_nt=10.0,
        min_samples=12,
        max_samples=400,
        max_iterations=40,
        damping=1.0e-2,
    ):
        self.soft_m = float(soft_m)
        self.min_anomaly_nt = float(min_anomaly_nt)
        self.min_samples = int(min_samples)
        self.max_samples = int(max_samples)
        self.max_iterations = int(max_iterations)
        self.damping = float(damping)
        self._samples = deque(maxlen=self.max_samples)

    def clear(self):
        self._samples.clear()

    def __len__(self):
        return len(self._samples)

    def add_sample(self, x, y, anomaly_nt):
        """Keep only calibrated-strength samples useful for the fit."""
        a = abs(float(anomaly_nt))
        if a < self.min_anomaly_nt:
            return False
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(a)):
            return False
        self._samples.append((float(x), float(y), a))
        return True

    def fit(self, guess_xy=None, guess_strength_nt=None):
        """
        Levenberg–Marquardt fit of A/(r^3+s^3) to buffered samples.

        Returns None if too few samples; otherwise a DipoleFix (success may
        still be False if the solver fails to improve).
        """
        if len(self._samples) < self.min_samples:
            return None

        samples = list(self._samples)
        tx, ty, strength = self._initial_guess(
            samples, guess_xy, guess_strength_nt
        )
        soft = self.soft_m
        lam = self.damping
        best = (tx, ty, strength)
        best_cost = self._cost(samples, tx, ty, strength, soft)

        for _ in range(self.max_iterations):
            jt_j, jt_r, cost = self._normal_equations(
                samples, tx, ty, strength, soft
            )
            if cost < best_cost:
                best_cost = cost
                best = (tx, ty, strength)

            # (JᵀJ + λ diag(JᵀJ)) δ = Jᵀ r   with r = model - measurement
            # We solve for δ that reduces residuals: minimize ||model-meas||
            # using δ = - (JᵀJ+λD)^{-1} Jᵀ residual, residual=model-meas.
            a00 = jt_j[0][0] * (1.0 + lam)
            a01 = jt_j[0][1]
            a02 = jt_j[0][2]
            a11 = jt_j[1][1] * (1.0 + lam)
            a12 = jt_j[1][2]
            a22 = jt_j[2][2] * (1.0 + lam)
            # Right-hand side: -Jᵀ residual
            b0 = -jt_r[0]
            b1 = -jt_r[1]
            b2 = -jt_r[2]

            delta = _solve_3x3(
                ((a00, a01, a02), (a01, a11, a12), (a02, a12, a22)),
                (b0, b1, b2),
            )
            if delta is None:
                lam *= 10.0
                continue

            d_tx, d_ty, d_s = delta
            # Bound steps so a bad iteration cannot jump across the lake.
            step = math.hypot(d_tx, d_ty)
            if step > 40.0:
                scale = 40.0 / step
                d_tx *= scale
                d_ty *= scale
                d_s *= scale

            trial_tx = tx + d_tx
            trial_ty = ty + d_ty
            trial_s = max(strength + d_s, 1.0)
            trial_cost = self._cost(
                samples, trial_tx, trial_ty, trial_s, soft
            )

            if trial_cost < cost:
                tx, ty, strength = trial_tx, trial_ty, trial_s
                lam = max(lam * 0.3, 1.0e-8)
                if math.hypot(d_tx, d_ty) < 1.0e-3 and abs(d_s) < 1.0:
                    break
            else:
                lam *= 8.0

        tx, ty, strength = best
        rms = math.sqrt(best_cost / max(len(samples), 1))
        # Require a meaningful improvement over "all mass at guess".
        success = rms < 20.0 and strength > 0.0
        return DipoleFix(
            x=tx,
            y=ty,
            strength_nt=strength,
            residual_rms_nt=rms,
            num_samples=len(samples),
            success=success,
        )

    def _initial_guess(self, samples, guess_xy, guess_strength_nt):
        if guess_xy is not None:
            tx, ty = float(guess_xy[0]), float(guess_xy[1])
        else:
            # Anomaly-weighted sample centroid as a warm start.
            wsum = 0.0
            xsum = 0.0
            ysum = 0.0
            for x, y, a in samples:
                wsum += a
                xsum += a * x
                ysum += a * y
            tx = xsum / wsum
            ty = ysum / wsum

        if guess_strength_nt is not None and guess_strength_nt > 0.0:
            strength = float(guess_strength_nt)
        else:
            # Invert A ≈ a * (r^3 + s^3) using the strongest sample.
            soft = self.soft_m
            best_a = -1.0
            strength = soft ** 3 * 50.0  # ~50 nT overhead default
            for x, y, a in samples:
                if a > best_a:
                    best_a = a
                    r = math.hypot(x - tx, y - ty)
                    strength = a * (r ** 3 + soft ** 3)
        return tx, ty, strength

    @staticmethod
    def _cost(samples, tx, ty, strength, soft):
        total = 0.0
        for x, y, a in samples:
            pred = model_anomaly_nt(x, y, tx, ty, strength, soft)
            err = pred - a
            total += err * err
        return total

    @staticmethod
    def _normal_equations(samples, tx, ty, strength, soft):
        """Build JᵀJ (3x3) and Jᵀ residual for Gauss-Newton."""
        jt_j = [[0.0, 0.0, 0.0] for _ in range(3)]
        jt_r = [0.0, 0.0, 0.0]
        cost = 0.0
        soft3 = soft * soft * soft

        for x, y, a in samples:
            dx = x - tx
            dy = y - ty
            r = math.hypot(dx, dy)
            denom = r * r * r + soft3
            if denom < 1.0e-12:
                continue
            pred = strength / denom
            residual = pred - a
            cost += residual * residual

            # pred = A/(r³+s³); ∂pred/∂r = -3 A r² / denom²
            # ∂r/∂tx = -dx/r ⇒ ∂pred/∂tx = 3 A r dx / denom²
            # (at r=0 soft term keeps pred finite; spatial grads vanish)
            if r > 1.0e-9:
                scale = strength * 3.0 * r / (denom * denom)
                j_tx = scale * dx
                j_ty = scale * dy
            else:
                j_tx = 0.0
                j_ty = 0.0
            j_a = 1.0 / denom
            jac = (j_tx, j_ty, j_a)

            for i in range(3):
                jt_r[i] += jac[i] * residual
                for j in range(i, 3):
                    jt_j[i][j] += jac[i] * jac[j]
        # Symmetrize
        jt_j[1][0] = jt_j[0][1]
        jt_j[2][0] = jt_j[0][2]
        jt_j[2][1] = jt_j[1][2]
        return jt_j, jt_r, cost


def _solve_3x3(a, b):
    """Solve A x = b for a symmetric 3x3 via Cramer's rule. None if singular."""
    (a00, a01, a02), (a10, a11, a12), (a20, a21, a22) = a
    det = (
        a00 * (a11 * a22 - a12 * a21)
        - a01 * (a10 * a22 - a12 * a20)
        + a02 * (a10 * a21 - a11 * a20)
    )
    if abs(det) < 1.0e-18:
        return None

    def det_replace(col):
        m = [
            [a00, a01, a02],
            [a10, a11, a12],
            [a20, a21, a22],
        ]
        for row in range(3):
            m[row][col] = b[row]
        return (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )

    return (
        det_replace(0) / det,
        det_replace(1) / det,
        det_replace(2) / det,
    )
