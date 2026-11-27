# Auditable ML advisory

AutonomyMind Decision keeps machine learning outside the safety authority.

The ML ranker may rank avoidance candidates only after each candidate has:
1. a dynamically feasible trajectory, and
2. passed every Full RTA gate: traffic, terrain, geofence, and flight envelope.

The operational authority remains deterministic:

```text
Sensors / state
      |
      v
Behavior / Decision Trees
      |
      v
LEFT / RIGHT / CLIMB candidates
      |
      v
Full RTA
      |
      +--> unsafe candidate: rejected before ML
      |
      v
auditable ML ranker (optional)
      |
      +--> confidence below threshold: deterministic fallback
      |
      v
MissionDecisionTree / Core output
```

## Runtime model

The deployed model is a transparent linear ranker stored in:

`config/ml/avoidance_ranker_v1.model`

The model contains:
- model name and version,
- feature-schema identifier,
- eight explicit feature weights,
- one bias for LEFT, RIGHT, and CLIMB.

`CoreConfig::load_strict()` parses the model as data, hashes its canonical
contents, and stores its coefficients and provenance in memory. Runtime
inference performs no model-file I/O.

The runtime features are deterministic normalized values derived from Full RTA
evidence:
- predicted minimum separation,
- terrain clearance,
- speed margin,
- vertical-speed margin,
- acceleration margin,
- jerk margin,
- yaw-rate margin,
- bank-angle margin.

The model cannot change thresholds or gate results.

The runtime confidence value is not a calibrated probability. It is the
logistic transform of the score margin between the two highest-ranked safe
candidates. The configured threshold is therefore an arbitration guard, not a
claim about statistical probability.

## Traceability

Every traffic-avoidance decision can expose:
- model name,
- model version,
- model SHA-256,
- feature-schema identifier,
- score for LEFT / RIGHT / CLIMB,
- proposed candidate,
- confidence,
- whether ML was used,
- whether the proposal was accepted,
- the final selected candidate,
- deterministic fallback reason.

This is stored in `MlTrace` alongside the existing `BehaviorTrace` and
`AvoidanceTrace`. Synthetic replay evidence and the dashboard also include
the ML provenance.

## Training

`tools/train_avoidance_ranker.py` trains the same transparent model format
from labelled CSV data.

Expected columns:

```text
maneuver,target_score,separation,terrain,speed_margin,vertical_margin,
accel_margin,jerk_margin,yaw_margin,bank_margin
```

Run:

```bash
python3 tools/train_avoidance_ranker.py \
  training.csv \
  trained.model \
  training_manifest.json
```

The manifest records the feature schema, sample count, dataset SHA-256, model
file SHA-256, feature names, maneuver classes, and trainer identifier.

A trained model is not activated merely because training completed. Deployment
requires replacing the configured model artifact, rerunning the complete
Decision test pipeline, Full RTA tests, sanitizer tests, rosbag replay, ROS 2
colcon tests, and Chromium dashboard checks.

## Fail-safe rules

- An unsafe candidate is never scored into eligibility.
- Non-finite model scores do not become commands.
- Low-confidence ML falls back to the deterministic minimum-separation selector.
- No safe candidate results in HOLD.
- Invalid Decision inputs still fail closed before arbitration.
- Disarmed state still produces STANDBY / NO_COMMAND.

The current model is a transparent bootstrap model intended to establish the
ML integration and audit contract. It is not represented as a flight-validated
or certified learned policy.
