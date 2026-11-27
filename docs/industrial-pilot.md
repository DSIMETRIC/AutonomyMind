# AutonomyMind Fleet — industrial inspection pilot

## Product hypothesis
Industrial operators and integrators need to coordinate heterogeneous autonomous fleets without rebuilding mission logic for each vehicle vendor. The initial proposed product is mission orchestration and auditable coordination in controlled sites, not an autonomous-driving product or certified safety controller. Validate willingness to pay through customer interviews and a paid pilot before committing to pricing.

## First runnable milestone
The offline scenario engine executes configuration-defined mission dependencies and capability-based task offers across arbitrary fleet identities. It records offer epochs, expiry, stale telemetry, reassignment, rejected late completions and shared conflict evidence. A synthetic fixture contains two aerial inspection members, a ground inspection member and a road logistics member; one aerial member stops reporting and its work is reassigned. Completion events are scripted inputs, not evidence of physical inspection. The coordinator is deterministic, single-process and transport-neutral; it does not implement AI inference, distributed consensus or actuator control. Offer expiry is an orchestration mechanism, not a physical safety guarantee; production integration needs acknowledgements, fencing at the vehicle, independent RTA and recovery. No third-party data used.

Run:
```bash
docker compose run --build --rm test
docker compose run --build --rm industrial-pilot
```

Output: semantic_data/outbox/industrial_pilot.json. The reference acceptance criteria are 3/3 synthetic tasks completed, one task reassigned to a different member and one stale completion rejected. Test repeatability and variable fleet sizes. These are expected results, not measured execution until run.

## Pilot stages and gates

| Stage | Deliverable | Acceptance |
|---|---|---|
| Offline coordination | Configured mission graph, leases, event replay | Deterministic replay; dependency ordering; stale epoch rejected; variable fleet sizes |
| Agent runtime | Registry and versioned agent contracts, local worker lifecycle, coordination transport | DDS integration tests, acknowledgements, task fencing, partition/recovery tests |
| Simulator integration | At least two drones and one rover, variable missions and obstacles | Measured task completion, collisions/contact, minimum separation, independent RTA results, network-fault experiments |
| Customer sandbox | Controlled-site integration with operator supervision | Agreed requirements, hazard analysis, independently reviewed test plan and explicit permission |
| Commercial product | Fleet dashboard, onboarding, audit export, deployment and support | Customer acceptance and commercial/legal/security review |

Record mission completion rate, time to reassign, per-agent availability, minimum measured separation, interventions, task latency and reproducibility. Physical metrics are not available in offline replay. Do not equate scripted completions with mission success in a simulator.

Commercial prerequisites: verify IP ownership and third-party licenses, security and privacy, applicable safety and regulatory requirements, integration responsibilities, support costs and customer demand. Waymo and other restricted datasets are excluded unless appropriate rights are established. The open-core packaging and pricing are hypotheses, not commitments.
