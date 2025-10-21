# Units and sign conventions

## Scope
This note records the design considerations for nonholonomic kinodynamic RRT* path planning and MPC tracking.

## Current project context
The repository currently implements curvature-fan steering, swept-arc collision checks, tree rewiring, curvature-limited speed profiles. The main code paths are `planner/kinodynamic_rrt_star.py` and `planner/mpc_tracker.py`. The principal engineering concerns are robot footprint inflation, collision geometry, heading wrap, goal tolerance, braking profile, path projection.

## Design questions
- What inputs, units, coordinate conventions, and preconditions does this part of the system require?
- Which state is authoritative, and which values are derived?
- How are invalid, stale, incomplete, or physically implausible inputs represented?
- Which assumptions are supported by source code or measured evidence, and which remain provisional?
- What observable evidence allows a reviewer to reproduce the behavior?

## Recommended design record
Document the selected approach, alternatives considered, rationale, expected failure modes, and any measurable trade-offs. Keep numerical constants linked to source or calibration evidence; do not copy unexplained values into a second source of truth.

## Verification
Pair this design note with the relevant scenarios under `docs/verification/scenarios/`. A design decision is not considered verified merely because the code compiles.

## Status
Working reference. Confirm implementation details against the source before using this note as a hardware, clinical, or production specification.
