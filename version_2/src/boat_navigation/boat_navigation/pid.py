"""PID controller with integral limits and conditional anti-windup."""

import math


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class PIDController:
    """Discrete PID with bounded output and anti-windup."""

    def __init__(
        self,
        kp,
        ki,
        kd,
        output_min,
        output_max,
        integral_limit,
        reset_error=math.pi,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit
        self.reset_error = reset_error
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0

        if not self.initialized:
            self.prev_error = error
            self.initialized = True

        if abs(error) > self.reset_error:
            self.integral = 0.0

        derivative = (error - self.prev_error) / dt
        unsat = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = clamp(unsat, self.output_min, self.output_max)

        if abs(unsat - output) < 1e-9:
            self.integral += error * dt
            self.integral = clamp(
                self.integral,
                -self.integral_limit,
                self.integral_limit,
            )
        elif math.copysign(1.0, error) == math.copysign(1.0, unsat):
            pass

        self.prev_error = error
        return output
