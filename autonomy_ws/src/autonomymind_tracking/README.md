# autonomymind_tracking

Deterministic 3-D multi-object tracking for fused AutonomyMind observations.

## Input

`/autonomymind/observations/fused`

Expected evidence:

- semantic label;
- 3-D position;
- covariance;
- camera/LiDAR provenance.

## Output

`/autonomymind/observations/tracks`

Each published observation contains:

- persistent `track_id`;
- filtered/estimated 3-D velocity;
- propagated diagonal covariance;
- track age and missed-update count;
- short-horizon predicted position;
- source observation provenance.

## Reference association

The public implementation uses deterministic nearest-neighbor association with:

- semantic-label compatibility;
- constant-velocity prediction;
- configurable 3-D gating radius;
- one-to-one assignment;
- configurable track expiry.

This is intentionally transparent and testable. A future probabilistic tracker or learned association model can replace the association logic without changing the Observation contract.
