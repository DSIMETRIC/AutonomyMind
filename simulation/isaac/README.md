# Isaac Sim Integration

This directory is the public integration surface for NVIDIA Isaac Sim.

Planned capabilities:

- robot and sensor configuration;
- scenario definitions;
- synthetic sensor generation;
- domain randomization;
- multi-robot cooperative scenarios;
- failure and degradation injection;
- regression evaluation;
- export of synthetic datasets into the semantic-data pipeline.

## Boundary

Isaac Sim remains an external dependency. AutonomyMind should not vendor proprietary simulator assets or components without explicit redistribution rights.

Simulation scenarios should be reproducible and versioned independently from production robot configuration.
