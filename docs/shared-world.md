# Shared Semantic World Model — evidence service reference

The shared world agent consumes versioned DDS envelopes for world.observation, world.member and scene.semantic, responds to world.query with world.snapshot, and acknowledges ingestion with world.receipt. Metric evidence requires a common world frame, source timestamp, position, velocity and uncertainty margin; observations also carry confidence and label. Identity is scoped to producer and kind, so equal local track IDs do not merge unrelated sources. Semantic VLM reports are stored separately and never converted into metric positions. The store is bounded, rejects stale or out-of-frame records, and expires records from snapshots. Empty or incomplete snapshots never establish free space or motion authorization.

Run on the trusted Linux host profile (stop conflicting manually launched workers first):

```bash
docker compose -f compose.agents.host.yaml build mission
docker compose -f compose.agents.host.yaml -f compose.agents.world.yaml up -d world
docker compose -f compose.agents.host.yaml -f compose.agents.world.yaml run --rm world-probe
```

The probe publishes synthetic observations for three producer identities and a synthetic semantic description, verifies a shared snapshot, then verifies expiry. These are not live sensor streams or actual multiple vehicles. Output: semantic_data/outbox/shared_world.json. The existing VLM publisher can send compatible scene reports, subject to the 2048-byte payload limit.

Current limitations: one in-memory service, not replicated or durable memory; no cross-source association, map fusion, SLAM, transforms, occupancy map exchange, probabilistic covariance fusion, mission-state synchronization or SemanticMap integration. No live planner/coordinator consumer yet. Trusted DDS identities are not authenticated; common synchronized clocks are required. Source provenance records declared sender and event identity, not proof of authenticity. Expired records are removed, not preserved as an audit history. Unit and DDS tests require execution. No motion or RTA authorization interface.
