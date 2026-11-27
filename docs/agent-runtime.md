# Agent Runtime — first executable reference

The runtime provides agent identity, role, version, capabilities, accepted/emitted event kinds, discovery, heartbeat expiry, correlation IDs, bounded event queues and delivery budgets, duplicate rejection, expiry and contract checks, handler fault isolation, and a delivery trace. The industrial pilot wraps the existing deterministic mission coordinator in mission, coordination and evaluation handlers. It uses synthetic events, not physical completion evidence.

Run:
```bash
docker compose run --build --rm test
docker compose run --build --rm agent-pilot
```
Output: semantic_data/outbox/agent_pilot.json.

This is an in-process synchronous reference transport with a logical clock. It does not provide authenticated network identities, persistent deduplication, distributed consensus, asynchronous process isolation, payload-specific schema validation, bounded handler execution time, transactional message delivery or AI inference. Heartbeats are explicit calls; there is no background timer. Trace and deduplication state are in memory. Agent code is trusted; no security sandbox is claimed. Handler failure does not roll back earlier emissions. No actuator interface exists.

Next: define per-event schemas and authenticated transport, implement process-local agent workers and ROS 2 DDS adapter, test network partitions and lease fencing, then connect world model and model-aware planner to independent RTA. Learned agents must propose through the same contracts; deterministic execution authority remains separate.
