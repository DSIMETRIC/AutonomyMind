# Core Agent Contracts

AutonomyMind agents exchange a small set of versioned, transport-independent contracts.

The first public contract set is:

- `Observation` — one structured perception/localization/fusion observation.
- `WorldState` — the shared semantic and kinematic state consumed by higher-level agents.
- `AgentProposal` — a candidate action, plan, or recommendation produced by an agent.
- `SafetyDecision` — deterministic acceptance/rejection evidence for a proposal.

## Design rules

1. Contracts are versioned independently from agent implementations.
2. Every payload carries timestamp, frame, provenance, source agent, and schema version.
3. Learned confidence is represented explicitly and never treated as proof of safety.
4. `SafetyDecision` is produced by deterministic validation/runtime-assurance logic.
5. The same logical contracts can be serialized as JSON, ROS 2 messages, logs, simulation events, or dataset records.

See the JSON Schemas in this directory and the ROS 2 message definitions under `autonomy_ws/src/autonomymind_interfaces/msg/`.
