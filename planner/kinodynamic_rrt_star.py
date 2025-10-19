"""Kinodynamic RRT* for a differential-drive / Dubins-constrained base.

State is (x, y, theta). Controls are (v, omega) sampled from the admissible
set defined by the curvature bound |omega| <= v / r_min. Collision checking
is done on the swept arc, not just the endpoint, otherwise the tree happily
cuts through pallet racking on tight turns.

Usage:
    python -m planner.kinodynamic_rrt_star --config configs/warehouse_corridor.json
"""

from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class State:
    x: float
    y: float
    theta: float


@dataclass
class Node:
    state: State
    parent: Optional[int]
    cost: float = 0.0
    # control that produced this node from its parent, needed for playback
    control: Tuple[float, float] = (0.0, 0.0)


@dataclass
class Polygon:
    vertices: np.ndarray

    def contains(self, p: np.ndarray) -> bool:
        """Ray casting. Assumes simple, non self-intersecting polygon."""
        n = len(self.vertices)
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = self.vertices[i]
            xj, yj = self.vertices[j]
            if (yi > p[1]) != (yj > p[1]):
                x_cross = (xj - xi) * (p[1] - yi) / (yj - yi) + xi
                if p[0] < x_cross:
                    inside = not inside
            j = i
        return inside


@dataclass
class Circle:
    cx: float
    cy: float
    r: float


@dataclass
class World:
    bounds: Tuple[float, float, float, float]  # xmin, ymin, xmax, ymax
    circles: List[Circle] = field(default_factory=list)
    polygons: List[Polygon] = field(default_factory=list)
    robot_radius: float = 0.32

    def in_bounds(self, x: float, y: float) -> bool:
        xmin, ymin, xmax, ymax = self.bounds
        return xmin <= x <= xmax and ymin <= y <= ymax

    def collides(self, x: float, y: float) -> bool:
        if not self.in_bounds(x, y):
            return True
        p = np.array([x, y])
        for c in self.circles:
            if math.hypot(x - c.cx, y - c.cy) < c.r + self.robot_radius:
                return True
        # polygon check uses an inflated test: sample the robot footprint ring
        for poly in self.polygons:
            if poly.contains(p):
                return True
            # cheap inflation by testing 8 points on the footprint circle
            for k in range(8):
                a = k * math.pi / 4.0
                q = p + self.robot_radius * np.array([math.cos(a), math.sin(a)])
                if poly.contains(q):
                    return True
        return False


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class KinodynamicRRTStar:
    """RRT* over the Dubins-like control set of a diff-drive base.

    Not a true kinodynamic planner (no acceleration bounds in the tree); the
    MPC tracker downstream is responsible for respecting accel limits. The
    naming is kept because that is what the internal roadmap calls it.
    """

    def __init__(
        self,
        world: World,
        r_min: float = 0.45,
        v_max: float = 0.9,
        step_arc: float = 0.35,
        goal_bias: float = 0.12,
        goal_tolerance: float = 0.25,
        seed: int = 7,
    ) -> None:
        self.world = world
        self.r_min = r_min
        self.v_max = v_max
        self.step_arc = step_arc
        self.goal_bias = goal_bias
        self.goal_tolerance = goal_tolerance
        self.rng = random.Random(seed)
        self.nodes: List[Node] = []
        self.goal: State = State(0.0, 0.0, 0.0)

    # ---------------------------------------------------------------- steering

    def sample(self) -> State:
        if self.rng.random() < self.goal_bias:
            return self.goal
        xmin, ymin, xmax, ymax = self.world.bounds
        return State(
            self.rng.uniform(xmin, xmax),
            self.rng.uniform(ymin, ymax),
            self.rng.uniform(-math.pi, math.pi),
        )

    def steer(self, a: State, b: State) -> Optional[Tuple[State, Tuple[float, float], float]]:
        """Pick a curvature-bounded arc from a toward b.

        We try a small fan of curvatures and keep the one whose endpoint is
        closest to b. Cheaper than solving the Dubins set exactly and good
        enough for a 0.35 m step length.
        """
        dx, dy = b.x - a.x, b.y - a.y
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            return None

        arc = min(self.step_arc, dist)
        best = None
        best_err = float("inf")
        # curvature fan, symmetric about zero, clamped by r_min
        kappa_max = 1.0 / self.r_min
        for kappa in np.linspace(-kappa_max, kappa_max, 11):
            theta = a.theta
            # exact arc integration
            if abs(kappa) < 1e-6:
                nx = a.x + arc * math.cos(theta)
                ny = a.y + arc * math.sin(theta)
                ntheta = theta
            else:
                nx = a.x + (math.sin(theta + kappa * arc) - math.sin(theta)) / kappa
                ny = a.y - (math.cos(theta + kappa * arc) - math.cos(theta)) / kappa
                ntheta = wrap_angle(theta + kappa * arc)
            err = math.hypot(nx - b.x, ny - b.y) + 0.15 * abs(wrap_angle(ntheta - b.theta))
            if err < best_err:
                best_err = err
                v = self.v_max
                best = (State(nx, ny, ntheta), (v, v * kappa), arc)
        if best is None:
            return None

        state, control, length = best
        if not self._arc_is_free(a, state, control[1]):
            return None
        return state, control, length

    def _arc_is_free(self, a: State, b: State, kappa: float) -> bool:
        """Swept-arc collision check. 6 samples is enough at these step sizes."""
        n = 6
        for i in range(n + 1):
            t = i / n
            if abs(kappa) < 1e-6:
                x = a.x + t * (b.x - a.x)
                y = a.y + t * (b.y - a.y)
            else:
                theta = a.theta
                seg = math.hypot(b.x - a.x, b.y - a.y) * t
                x = a.x + (math.sin(theta + kappa * seg) - math.sin(theta)) / kappa
                y = a.y - (math.cos(theta + kappa * seg) - math.cos(theta)) / kappa
            if self.world.collides(x, y):
                return False
        return True

    # ------------------------------------------------------------------- tree

    def nearest(self, s: State) -> int:
        best_i, best_d = 0, float("inf")
        for i, n in enumerate(self.nodes):
            d = math.hypot(n.state.x - s.x, n.state.y - s.y)
            d += 0.2 * abs(wrap_angle(n.state.theta - s.theta))
            if d < best_d:
                best_i, best_d = i, d
        return best_i

    def near(self, s: State, radius: float) -> Sequence[int]:
        return [
            i
            for i, n in enumerate(self.nodes)
            if math.hypot(n.state.x - s.x, n.state.y - s.y) <= radius
        ]

    def plan(self, start: State, goal: State, max_iters: int = 4000) -> List[Node]:
        self.goal = goal
        self.nodes = [Node(start, None, 0.0)]

        for _ in range(max_iters):
            q_rand = self.sample()
            i_near = self.nearest(q_rand)
            steered = self.steer(self.nodes[i_near].state, q_rand)
            if steered is None:
                continue
            q_new, control, length = steered

            # choose parent by cost-to-come, not by distance
            radius = min(1.2, 2.0 * self.step_arc)
            best_parent = i_near
            best_cost = self.nodes[i_near].cost + length
            for j in self.near(q_new, radius):
                cand = self.steer(self.nodes[j].state, q_new)
                if cand is None:
                    continue
                _, _, l = cand
                c = self.nodes[j].cost + l
                if c < best_cost:
                    best_parent, best_cost = j, c

            idx = len(self.nodes)
            self.nodes.append(Node(q_new, best_parent, best_cost, control))

            # rewire: if routing through the new node is cheaper, reparent
            for j in self.near(q_new, radius):
                if j == best_parent:
                    continue
                cand = self.steer(q_new, self.nodes[j].state)
                if cand is None:
                    continue
                _, _, l = cand
                if best_cost + l < self.nodes[j].cost - 1e-9:
                    self.nodes[j].parent = idx
                    self.nodes[j].cost = best_cost + l

            if math.hypot(q_new.x - goal.x, q_new.y - goal.y) < self.goal_tolerance:
                return self._extract(idx)

        # TODO: add a proper Dubins-shot connection to the goal so we stop
        # relying on the goal bias to stumble into the tolerance ball.
        return []

    def _extract(self, idx: int) -> List[Node]:
        path: List[Node] = []
        while idx is not None:
            path.append(self.nodes[idx])
            idx = self.nodes[idx].parent
        path.reverse()
        return path


def load_world(path: str) -> Tuple[World, State, State, dict]:
    with open(path, "r", encoding="utf-8") as fh:
        cfg = json.load(fh)

    w = cfg["world"]
    world = World(
        bounds=tuple(w["bounds"]),
        circles=[Circle(c["cx"], c["cy"], c["r"]) for c in w.get("circles", [])],
        polygons=[Polygon(np.array(p, dtype=float)) for p in w.get("polygons", [])],
        robot_radius=w.get("robot_radius", 0.32),
    )
    s = cfg["start"]
    g = cfg["goal"]
    start = State(s["x"], s["y"], s["theta"])
    goal = State(g["x"], g["y"], g["theta"])
    return world, start, goal, cfg


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/warehouse_corridor.json")
    args = ap.parse_args()

    world, start, goal, cfg = load_world(args.config)
    planner = KinodynamicRRTStar(
        world,
        r_min=cfg["curvature"]["min_turn_radius_m"],
        v_max=cfg["limits"]["v_max_mps"],
    )
    path = planner.plan(start, goal)
    if not path:
        print("no solution within iteration budget")
        return
    print(f"nodes={len(planner.nodes)} path_waypoints={len(path)} cost={path[-1].cost:.3f}")
    for n in path[::5]:
        print(f"  {n.state.x:7.3f} {n.state.y:7.3f} {n.state.theta:6.3f}")


if __name__ == "__main__":
    main()
