"""Least-squares dipole localization from (x, y, anomaly) samples.

Two models (see ``dipole_model``):

* ``scalar_soft`` -- legacy A/(r^3+s^3) fit of (tx, ty, A).
* ``total_field`` -- vector dipole with full |B0+Banom|-|B0| anomaly
  fit of (tx, ty[, tz], mx, my, mz). Bayes/MI still consume scalar |a|;
  this fitter only refines the continuous fix.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

from boat_sensing.dipole import (
    dipole_B_nt,
    earth_field_enu,
    full_total_field_delta_nt,
    total_field_anomaly_nt,
    unit_vector,
    vertical_moment_for_peak_nt,
)


def model_anomaly_nt(x, y, tx, ty, strength_nt, soft_m):
    """A / (r^3 + soft^3) — same form as the planted scalar_soft dipole."""
    dx = float(x) - float(tx)
    dy = float(y) - float(ty)
    r = math.hypot(dx, dy)
    soft = max(float(soft_m), 1e-3)
    denom = r * r * r + soft * soft * soft
    return float(strength_nt) / denom


def model_total_field_nt(
    x,
    y,
    tx,
    ty,
    tz,
    mx,
    my,
    mz,
    f_hat,
    sensor_z=0.0,
    fit_abs=True,
    soft_m=0.0,
    earth_total_nt=45000.0,
    inclination_deg=15.0,
    declination_deg=-1.0,
    use_full_delta=True,
):
    """Predicted total-field anomaly (optionally absolute) at a surface sample."""
    bd = dipole_B_nt(
        float(x) - float(tx),
        float(y) - float(ty),
        float(sensor_z) - float(tz),
        float(mx),
        float(my),
        float(mz),
        soft_m=soft_m,
    )
    if use_full_delta:
        dF = full_total_field_delta_nt(
            bd[0],
            bd[1],
            bd[2],
            earth_total_nt,
            inclination_deg,
            declination_deg,
        )
    else:
        dF = total_field_anomaly_nt(bd[0], bd[1], bd[2], f_hat)
    return abs(dF) if fit_abs else dF


@dataclass
class DipoleFix:
    """Result of a dipole least-squares localization."""

    x: float
    y: float
    strength_nt: float
    residual_rms_nt: float
    num_samples: int
    success: bool
    z: float = 0.0
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0
    model: str = 'scalar_soft'


class DipoleFitter:
    """Buffer strong anomaly samples and fit a continuous dipole."""

    def __init__(
        self,
        soft_m=20.0,
        min_anomaly_nt=10.0,
        min_samples=12,
        max_samples=400,
        max_iterations=40,
        damping=1.0e-2,
        dipole_model='scalar_soft',
        target_z=-1.0,
        sensor_z=0.0,
        earth_inclination_deg=15.0,
        earth_declination_deg=-1.0,
        earth_total_nt=45000.0,
        free_depth=False,
        fit_abs=True,
        guess_peak_nt=50.0,
        free_moment=True,
        xy_warmup_iterations=15,
    ):
        self.soft_m = float(soft_m)
        self.min_anomaly_nt = float(min_anomaly_nt)
        self.min_samples = int(min_samples)
        self.max_samples = int(max_samples)
        self.max_iterations = int(max_iterations)
        self.damping = float(damping)
        self.dipole_model = str(dipole_model).strip().lower()
        if self.dipole_model not in ('scalar_soft', 'total_field'):
            self.dipole_model = 'scalar_soft'
        self.target_z = float(target_z)
        self.sensor_z = float(sensor_z)
        self.earth_inclination_deg = float(earth_inclination_deg)
        self.earth_declination_deg = float(earth_declination_deg)
        self.earth_total_nt = float(earth_total_nt)
        self.free_depth = bool(free_depth)
        self.fit_abs = bool(fit_abs)
        self.guess_peak_nt = float(guess_peak_nt)
        self.free_moment = bool(free_moment)
        self.xy_warmup_iterations = int(xy_warmup_iterations)
        self._m_scale = 1.0e6
        self._f_hat = unit_vector(
            *earth_field_enu(
                self.earth_total_nt,
                self.earth_inclination_deg,
                self.earth_declination_deg,
            )
        )
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

    def fit(self, guess_xy=None, guess_strength_nt=None, guess_peak_nt=None):
        """
        Levenberg–Marquardt dipole fit to buffered samples.

        Returns None if too few samples; otherwise a DipoleFix (success may
        still be False if the solver fails to improve).
        """
        if len(self._samples) < self.min_samples:
            return None
        if self.dipole_model == 'total_field':
            return self._fit_total_field(
                guess_xy=guess_xy,
                guess_peak_nt=guess_peak_nt,
            )
        return self._fit_scalar_soft(
            guess_xy=guess_xy,
            guess_strength_nt=guess_strength_nt,
        )

    def _predict_total_field(self, x, y, tx, ty, tz, mx, my, mz):
        return model_total_field_nt(
            x,
            y,
            tx,
            ty,
            tz,
            mx,
            my,
            mz,
            self._f_hat,
            sensor_z=self.sensor_z,
            fit_abs=self.fit_abs,
            soft_m=self.soft_m,
            earth_total_nt=self.earth_total_nt,
            inclination_deg=self.earth_inclination_deg,
            declination_deg=self.earth_declination_deg,
            use_full_delta=True,
        )

    def _fit_scalar_soft(self, guess_xy, guess_strength_nt):
        samples = list(self._samples)
        tx, ty, strength = self._initial_guess(
            samples, guess_xy, guess_strength_nt
        )
        soft = self.soft_m
        lam = self.damping
        best = (tx, ty, strength)
        best_cost = self._cost_scalar(samples, tx, ty, strength, soft)

        for _ in range(self.max_iterations):
            jt_j, jt_r, cost = self._normal_equations_scalar(
                samples, tx, ty, strength, soft
            )
            if cost < best_cost:
                best_cost = cost
                best = (tx, ty, strength)

            a00 = jt_j[0][0] * (1.0 + lam)
            a01 = jt_j[0][1]
            a02 = jt_j[0][2]
            a11 = jt_j[1][1] * (1.0 + lam)
            a12 = jt_j[1][2]
            a22 = jt_j[2][2] * (1.0 + lam)
            delta = _solve_linear(
                [
                    [a00, a01, a02],
                    [a01, a11, a12],
                    [a02, a12, a22],
                ],
                [-jt_r[0], -jt_r[1], -jt_r[2]],
            )
            if delta is None:
                lam *= 10.0
                continue

            d_tx, d_ty, d_s = delta
            step = math.hypot(d_tx, d_ty)
            if step > 40.0:
                scale = 40.0 / step
                d_tx *= scale
                d_ty *= scale
                d_s *= scale

            trial_tx = tx + d_tx
            trial_ty = ty + d_ty
            trial_s = max(strength + d_s, 1.0)
            trial_cost = self._cost_scalar(
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
        success = rms < 20.0 and strength > 0.0
        return DipoleFix(
            x=tx,
            y=ty,
            z=0.0,
            strength_nt=strength,
            residual_rms_nt=rms,
            num_samples=len(samples),
            success=success,
            model='scalar_soft',
        )

    def _unpack_scaled(self, params, freeze_moment=False, m_frozen=None):
        """Decode optimizer params to physical (tx, ty, tz, mx, my, mz)."""
        tx, ty = params[0], params[1]
        idx = 2
        tz = self.target_z
        if self.free_depth:
            tz = params[2]
            idx = 3
        if freeze_moment:
            mx, my, mz = m_frozen
        else:
            s = self._m_scale
            mx = params[idx] * s
            my = params[idx + 1] * s
            mz = params[idx + 2] * s
        return tx, ty, tz, mx, my, mz

    def _pack_scaled(self, tx, ty, tz, mx, my, mz, freeze_moment=False):
        params = [tx, ty]
        if self.free_depth:
            params.append(tz)
        if not freeze_moment:
            s = max(self._m_scale, 1.0)
            params.extend([mx / s, my / s, mz / s])
        return params

    def _fit_total_field(self, guess_xy, guess_peak_nt):
        samples = list(self._samples)
        peak = (
            float(guess_peak_nt)
            if guess_peak_nt is not None and guess_peak_nt > 0.0
            else self.guess_peak_nt
        )
        if guess_xy is not None:
            tx, ty = float(guess_xy[0]), float(guess_xy[1])
        else:
            wsum = 0.0
            xsum = 0.0
            ysum = 0.0
            for x, y, a in samples:
                wsum += a
                xsum += a * x
                ysum += a * y
            tx = xsum / max(wsum, 1.0e-9)
            ty = ysum / max(wsum, 1.0e-9)
        tz = self.target_z
        mx, my, mz = vertical_moment_for_peak_nt(
            peak,
            tz,
            self.earth_inclination_deg,
            sensor_z=self.sensor_z,
            soft_m=self.soft_m,
            earth_total_nt=self.earth_total_nt,
            declination_deg=self.earth_declination_deg,
            use_full_delta=True,
        )
        self._m_scale = max(math.sqrt(mx * mx + my * my + mz * mz), 1.0e3)

        # Stage 1: xy (and optional tz) with m frozen — well-conditioned.
        warmup_iters = max(0, self.xy_warmup_iterations)
        params = self._pack_scaled(tx, ty, tz, mx, my, mz, freeze_moment=True)
        params, best_cost = self._lm_loop(
            samples,
            params,
            iterations=warmup_iters,
            freeze_moment=True,
            m_frozen=(mx, my, mz),
        )
        tx, ty, tz, mx, my, mz = self._unpack_scaled(
            params, freeze_moment=True, m_frozen=(mx, my, mz)
        )

        # Stage 2: thaw scaled moment (and depth if enabled).
        if self.free_moment or self.free_depth:
            params = self._pack_scaled(
                tx, ty, tz, mx, my, mz, freeze_moment=not self.free_moment
            )
            rest = max(self.max_iterations - warmup_iters, 20)
            params, best_cost = self._lm_loop(
                samples,
                params,
                iterations=rest,
                freeze_moment=not self.free_moment,
                m_frozen=None if self.free_moment else (mx, my, mz),
            )
            tx, ty, tz, mx, my, mz = self._unpack_scaled(
                params,
                freeze_moment=not self.free_moment,
                m_frozen=None if self.free_moment else (mx, my, mz),
            )

        rms = math.sqrt(best_cost / max(len(samples), 1))
        strength = math.sqrt(mx * mx + my * my + mz * mz)
        success = rms < 20.0 and strength > 0.0
        return DipoleFix(
            x=tx,
            y=ty,
            z=tz,
            strength_nt=strength,
            residual_rms_nt=rms,
            num_samples=len(samples),
            success=success,
            mx=mx,
            my=my,
            mz=mz,
            model='total_field',
        )

    def _lm_loop(self, samples, params, iterations, freeze_moment, m_frozen):
        lam = self.damping
        best = list(params)
        best_cost = self._cost_total_field(
            samples, params, freeze_moment=freeze_moment, m_frozen=m_frozen
        )
        for _ in range(max(0, iterations)):
            jt_j, jt_r, cost = self._normal_equations_total_field(
                samples,
                params,
                freeze_moment=freeze_moment,
                m_frozen=m_frozen,
            )
            if cost < best_cost:
                best_cost = cost
                best = list(params)

            n = len(params)
            damped = [
                [
                    jt_j[i][j] * (1.0 + lam if i == j else 1.0)
                    for j in range(n)
                ]
                for i in range(n)
            ]
            delta = _solve_linear(damped, [-jt_r[i] for i in range(n)])
            if delta is None:
                lam *= 10.0
                continue

            delta = self._clip_total_field_step(
                params, delta, freeze_moment=freeze_moment
            )
            trial = [params[i] + delta[i] for i in range(n)]
            trial = self._clamp_total_field_params(trial)
            trial_cost = self._cost_total_field(
                samples, trial, freeze_moment=freeze_moment, m_frozen=m_frozen
            )

            if trial_cost < cost:
                params = trial
                lam = max(lam * 0.3, 1.0e-8)
                xy_step = math.hypot(delta[0], delta[1])
                rest = math.sqrt(sum(d * d for d in delta[2:])) if n > 2 else 0.0
                if xy_step < 1.0e-3 and rest < 1.0e-4:
                    break
            else:
                lam *= 8.0

        if self._cost_total_field(
            samples, params, freeze_moment=freeze_moment, m_frozen=m_frozen
        ) < best_cost:
            best = list(params)
            best_cost = self._cost_total_field(
                samples, params, freeze_moment=freeze_moment, m_frozen=m_frozen
            )
        return best, best_cost

    def _clip_total_field_step(self, params, delta, freeze_moment=False):
        delta = list(delta)
        xy = math.hypot(delta[0], delta[1])
        if xy > 40.0:
            scale = 40.0 / xy
            for i in range(len(delta)):
                delta[i] *= scale
        idx = 2
        if self.free_depth:
            delta[2] = max(-8.0, min(8.0, delta[2]))
            idx = 3
        if not freeze_moment and len(delta) > idx:
            dm = math.sqrt(sum(d * d for d in delta[idx:idx + 3]))
            if dm > 0.35:
                scale = 0.35 / dm
                delta[idx] *= scale
                delta[idx + 1] *= scale
                delta[idx + 2] *= scale
        return delta

    def _clamp_total_field_params(self, params):
        params = list(params)
        if self.free_depth:
            z_lo = self.sensor_z - 80.0
            z_hi = self.sensor_z - 0.5
            params[2] = min(max(params[2], z_lo), z_hi)
        return params

    def _cost_total_field(self, samples, params, freeze_moment=False, m_frozen=None):
        tx, ty, tz, mx, my, mz = self._unpack_scaled(
            params, freeze_moment=freeze_moment, m_frozen=m_frozen
        )
        total = 0.0
        for x, y, a in samples:
            pred = self._predict_total_field(x, y, tx, ty, tz, mx, my, mz)
            err = pred - a
            total += err * err
        return total

    def _normal_equations_total_field(
        self, samples, params, freeze_moment=False, m_frozen=None
    ):
        n = len(params)
        eps = self._fd_eps(params, freeze_moment=freeze_moment)
        jt_j = [[0.0] * n for _ in range(n)]
        jt_r = [0.0] * n
        cost = 0.0
        for x, y, a in samples:
            pred = self._eval_params(
                x, y, params, freeze_moment=freeze_moment, m_frozen=m_frozen
            )
            residual = pred - a
            cost += residual * residual
            jac = [0.0] * n
            for k in range(n):
                bumped = list(params)
                bumped[k] += eps[k]
                jac[k] = (
                    self._eval_params(
                        x,
                        y,
                        bumped,
                        freeze_moment=freeze_moment,
                        m_frozen=m_frozen,
                    )
                    - pred
                ) / eps[k]
            for i in range(n):
                jt_r[i] += jac[i] * residual
                for j in range(i, n):
                    jt_j[i][j] += jac[i] * jac[j]
        for i in range(n):
            for j in range(i):
                jt_j[i][j] = jt_j[j][i]
        return jt_j, jt_r, cost

    def _eval_params(self, x, y, params, freeze_moment=False, m_frozen=None):
        tx, ty, tz, mx, my, mz = self._unpack_scaled(
            params, freeze_moment=freeze_moment, m_frozen=m_frozen
        )
        return self._predict_total_field(x, y, tx, ty, tz, mx, my, mz)

    def _fd_eps(self, params, freeze_moment=False):
        eps = [0.05, 0.05]
        idx = 2
        if self.free_depth:
            eps.append(0.05)
            idx = 3
        if not freeze_moment and len(params) > idx:
            eps.extend([1.0e-3, 1.0e-3, 1.0e-3])
        return eps

    def _initial_guess(self, samples, guess_xy, guess_strength_nt):
        if guess_xy is not None:
            tx, ty = float(guess_xy[0]), float(guess_xy[1])
        else:
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
            soft = self.soft_m
            best_a = -1.0
            strength = soft ** 3 * 50.0
            for x, y, a in samples:
                if a > best_a:
                    best_a = a
                    r = math.hypot(x - tx, y - ty)
                    strength = a * (r ** 3 + soft ** 3)
        return tx, ty, strength

    @staticmethod
    def _cost_scalar(samples, tx, ty, strength, soft):
        total = 0.0
        for x, y, a in samples:
            pred = model_anomaly_nt(x, y, tx, ty, strength, soft)
            err = pred - a
            total += err * err
        return total

    @staticmethod
    def _normal_equations_scalar(samples, tx, ty, strength, soft):
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
        jt_j[1][0] = jt_j[0][1]
        jt_j[2][0] = jt_j[0][2]
        jt_j[2][1] = jt_j[1][2]
        return jt_j, jt_r, cost


def _solve_linear(matrix, rhs):
    """Gaussian elimination with partial pivoting. None if singular."""
    n = len(rhs)
    if n == 0 or any(len(row) != n for row in matrix):
        return None
    aug = [list(matrix[i]) + [float(rhs[i])] for i in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(aug[row][col]))
        if abs(aug[pivot][col]) < 1.0e-18:
            return None
        if pivot != col:
            aug[col], aug[pivot] = aug[pivot], aug[col]
        div = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] /= div
        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            if factor == 0.0:
                continue
            for j in range(col, n + 1):
                aug[row][j] -= factor * aug[col][j]
    return [aug[i][n] for i in range(n)]


def _solve_3x3(a, b):
    """Solve A x = b for a 3x3. Kept for callers/tests; uses _solve_linear."""
    return _solve_linear([list(row) for row in a], list(b))
