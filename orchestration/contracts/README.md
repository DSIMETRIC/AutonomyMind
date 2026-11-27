# Agent Contracts

AutonomyMind agents communicate through explicit, versioned contracts.

Minimum metadata for agent outputs should include:

- `agent_id`
- `agent_version`
- `timestamp`
- `frame_id`
- `source`
- `confidence` or uncertainty representation when applicable
- `provenance`
- `schema_version`

Decision-support outputs should additionally record:

- candidate set;
- selected or ranked proposal;
- supporting evidence;
- fallback reason;
- model or policy version;
- whether deterministic safety accepted or rejected the proposal.

Contracts should remain transport-independent so they can be mapped to ROS 2 messages, simulation APIs, logs, or offline datasets.
