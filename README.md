# nonholonomic-rrt-star-planner

Kinodynamic RRT* plus a cross-track tracker for a differential-drive base with
a curvature bound. Written against a warehouse corridor map, but the world
model is generic (circles + simple polygons, inflated by the robot radius).

## Layout

- `planner/kinodynamic_rrt_star.py` -- planner. Curvature-fan steering, swept
  arc collision checks, cost-to-come parent selection, rewiring.
- `planner/mpc_tracker.py` -- path projection, curvature-limited speed
  profile with a backward braking pass, linearised cross-track feedback.
- `configs/warehouse_corridor.json` -- DC-2 aisle 14 map.

## Run

```
pip install -r requirements.txt
python -m planner.kinodynamic_rrt_star --config configs/warehouse_corridor.json
```

## Notes

Steering picks from an 11-sample curvature fan instead of solving the Dubins
set. Cheaper, and the endpoint error is inside the goal tolerance anyway.
Acceleration limits are not in the tree -- the tracker owns them. Replanning is
not implemented; the plan is assumed valid for one traversal.
