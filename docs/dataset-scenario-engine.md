# Dataset and Scenario Engine — proposed design

Status: design only. No Waymo importer or dataset downloaded.

The engine will expose a source-neutral scenario reader to Dataset, Evaluation, World Model and Prediction Agents. Start with small authorized offline samples, not a full dataset download.

## Adapter contract

- Source identity, version, split, license identifier/terms URL, permitted purpose, consent or approval record where applicable.
- Scenario identity, time base, frame convention, ego/actor IDs, observation streams, map/lane geometry, trajectories, validity masks and provenance.
- Explicit separation between measured observations, annotations/ground truth and predictions.
- Replay cursor, deterministic ordering, bounded reads, schema validation and missing-data handling.
- No credentials, source data or restricted derived artifacts committed to the repository.

## Candidate sources

Waymo Open Dataset Motion (trajectory and map replay), Perception (sensor data), and End-to-End Driving (driving evaluation) are potential research adapters subject to the [applicable terms](https://waymo.com/open/terms/). Obtain appropriate rights before company or commercial use; do not assume public accessibility means commercial permission. Traffic actors are not labeled cooperative autonomous agents. Synthetic fleet data and appropriately licensed V2X datasets are complementary.

## Implementation plan

1. Define canonical scenario schema and synthetic fixtures with provenance.
2. Implement a bounded local reader and replay iterator.
3. Add a Waymo Motion adapter behind explicit license/purpose approval and optional dependencies.
4. Evaluate prediction and planning offline without treating labels as live sensor inputs.
5. Export generic annotation/evaluation contracts to SemanticMap-compatible workflows.
6. Record dataset and model versions and enforce human review before deployment.

Tests must cover frame/time conversion, invalid/missing states, identity consistency, variable actor count, reproducibility, licensing metadata and isolation from actuator topics. Offline evaluation does not authorize real-world operation.
