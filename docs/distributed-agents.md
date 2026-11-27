# Distributed Agent Runtime — ROS 2 DDS reference

Three independent containers exchange versioned JSON envelopes over a ROS 2 String topic using reliable DDS QoS. Mission and coordination roles discover peers through periodic capability heartbeats with local receipt leases. Evaluation writes an offline report; no motion topic or RTA authorization is exposed. Runtime role identities are configurable by CLI; Compose provides three example instances.

```bash
docker compose -f compose.agents.yaml build mission
docker compose -f compose.agents.yaml up -d mission coordination evaluation
docker compose -f compose.agents.yaml run --rm probe
docker compose -f compose.agents.yaml stop coordination
docker compose -f compose.agents.yaml run --rm probe --scenario /workspace/pilots/industrial_inspection.json --expect-no-coordinator
docker compose -f compose.agents.yaml start coordination
docker compose -f compose.agents.yaml run --rm probe
docker compose -f compose.agents.yaml down
```

Output: semantic_data/outbox/dds_pilot.json. The probe tests message delivery and missing coordinator discovery; the mission report contains synthetic task completion events, not live physical task execution. A fresh probe does not establish failover during an active mission. Unit tests cover duplicate, expiry, contract and heartbeat replay rejection. This implementation requires synchronized host clocks, a trusted internal Docker network, and working DDS multicast. No authenticated identity, persistent deduplication, distributed consensus, in-flight retries, task fencing, AI inference, or payload-specific schema validation. The separate in-process reference runtime remains intact. No simulator or physical actuator connection. Test locally before claiming successful DDS integration.
