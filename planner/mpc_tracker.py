"""Cross-track error tracker for a planned (x, y, theta) waypoint list.

The "MPC" here is a single-step linearized receding horizon over the
cross-track / heading error model, which is all a diff-drive needs at these
speeds. Velocity is profiled against curvature and remaining path length so
we do not roll the robot over in the 90-degree corridor turns.

Error model, small-angle, arc-length parameterised:
    e_y'     = sin(psi)
    psi'     = omega / v - kappa_ref
with kappa_ref the curvature of the reference path at the projection point.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np


@dataclass
class Waypoint:
    x: float
    y: float
    theta: float
    kappa: float = 0.0


@dataclass
class TrackerConfig:
    wheelbase: float = 0.62
    v_nominal: float = 0.55
    v_min: float = 0.05
    a_max: float = 0.7
    omega_max: float = 1.4
    lookahead: float = 0.55
    k_cross: float = 1.9
    k_heading: float = 2.4
    dt: float = 0.05


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def cumulative_length(path: Sequence[Waypoint]) -> np.ndarray:
    s = [0.0]
    for a, b in zip(path, path[1:]):
        s.append(s[-1] + math.hypot(b.x - a.x, b.y - a.y))
    return np.asarray(s)


def project(path: Sequence[Waypoint], s: np.ndarray, x: float, y: float) -> Tuple[float, float, int]:
    """Project the pose onto the polyline. Returns (s_star, e_y, segment index).

    Brute force over segments. Path lengths here are tens of metres, so this
    is a few microseconds; do not optimise it until the profile says so.
    """
    best = (0.0, 0.0, 0)
    best_d = float("inf")
    for i in range(len(path) - 1):
        ax, ay = path[i].x, path[i].y
        bx, by = path[i + 1].x, path[i + 1].y
        vx, vy = bx - ax, by - ay
        seg_len2 = vx * vx + vy * vy
        if seg_len2 < 1e-9:
            continue
        t = ((x - ax) * vx + (y - ay) * vy) / seg_len2
        t = min(1.0, max(0.0, t))
        px, py = ax + t * vx, ay + t * vy
        d = math.hypot(x - px, y - py)
        if d < best_d:
            # signed lateral error, left positive
            cross = vx * (y - ay) - vy * (x - ax)
            sign = 1.0 if cross >= 0 else -1.0
            best = (s[i] + t * math.sqrt(seg_len2), sign * d, i)
            best_d = d
    return best


def speed_profile(path: Sequence[Waypoint], s: np.ndarray, cfg: TrackerConfig) -> np.ndarray:
    """Curvature-limited speed, then a backward pass for the stop at the end."""
    n = len(path)
    v = np.full(n, cfg.v_nominal)
    for i, wp in enumerate(path):
        kappa = abs(wp.kappa) + 1e-6
        # lateral accel budget ~ 0.9 m/s^2
        v_lat = math.sqrt(0.9 / kappa)
        v[i] = min(v[i], max(cfg.v_min, v_lat))

    # backward pass: v_i^2 <= v_{i+1}^2 + 2 a ds
    for i in range(n - 2, -1, -1):
        ds = max(s[i + 1] - s[i], 1e-3)
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * cfg.a_max * ds))
    return v


class MPCTracker:
    def __init__(self, path: List[Waypoint], cfg: TrackerConfig | None = None) -> None:
        self.path = path
        self.cfg = cfg or TrackerConfig()
        self.s = cumulative_length(path)
        self.v_ref = speed_profile(path, self.s, self.cfg)
        self.v_cmd = 0.0

    def command(self, x: float, y: float, theta: float) -> Tuple[float, float]:
        cfg = self.cfg
        s_star, e_y, seg = project(self.path, self.s, x, y)

        # reference heading from the local segment
        wp = self.path[seg]
        theta_ref = wp.theta
        psi = wrap(theta_ref - theta)

        # lookahead target for the feed-forward curvature
        s_target = min(s_star + cfg.lookahead, self.s[-1])
        idx = int(np.searchsorted(self.s, s_target, side="right") - 1)
        idx = max(0, min(idx, len(self.path) - 1))
        kappa_ff = self.path[idx].kappa

        # linearised feedback: omega = v * (kappa_ff + k_y e_y + k_psi sin(psi))
        v = float(np.interp(s_star, self.s, self.v_ref))
        omega = v * (
            kappa_ff
            + cfg.k_cross * e_y
            + cfg.k_heading * math.sin(psi) / max(cfg.lookahead, 1e-3)
        )

        # rate limit so the MPC handoff to the base firmware is not a step input
        omega = max(-cfg.omega_max, min(cfg.omega_max, omega))
        dv = cfg.a_max * cfg.dt
        self.v_cmd = max(self.v_cmd - dv, min(self.v_cmd + dv, v))

        # slow down when the heading error is large; turning while fast is
        # how we ended up with the scuffed bumpers last quarter
        if abs(psi) > 0.6:
            self.v_cmd *= 0.6

        return self.v_cmd, omega

    def finished(self, x: float, y: float) -> bool:
        end = self.path[-1]
        return math.hypot(x - end.x, y - end.y) < 0.12
