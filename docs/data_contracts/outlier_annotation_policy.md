# Outlier annotation policy

## Scope
Data handling reference for nonholonomic kinodynamic RRT* path planning and MPC tracking.

## Relevant sources
- Input/measurement source: inspect `planner/mpc_tracker.py` and the repository's existing data files.
- Consumer/algorithm: `planner/kinodynamic_rrt_star.py`.
- Domain concerns: robot footprint inflation, collision geometry, heading wrap, goal tolerance, braking profile, path projection.

## Contract requirements
- Preserve source values and original units.
- State column/field names, types, units, valid ranges, and whether a field is optional.
- Define timestamp origin, sampling assumptions, and ordering requirements where applicable.
- Distinguish missing, invalid, saturated, and intentionally masked values.
- Keep calibration identifiers and acquisition metadata alongside measurements.
- Reject malformed records with an actionable diagnostic; do not silently invent defaults.

## Provenance
For every derived dataset, record its source, transformation script, parameters, timestamp, and checksum. Mark simulated or synthetic data clearly and never present it as a measured capture.

## Review status
Template for project-specific completion. Confirm exact field names and constraints against the actual parser and checked-in fixtures before publishing a formal schema.
