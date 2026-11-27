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

## Linux host development profile

A workstation experiment on the DDS worker baseline (commit e2e5ee3) passed using Docker host networking, host IPC and ROS_DOMAIN_ID=79. The isolated bridge configuration had three running workers but only one publisher/subscriber visible from Mission. Host mode changes both network and IPC; the exact root cause was not isolated.

Reported probe output: 3/3 synthetic tasks completed, 4 members seen, 1 reassigned task, 1 revoked offer, 1 rejected completion, 0 conflict snapshots, 4 unknown-peer snapshots. PASS: mission -> coordination -> evaluation over ROS 2 DDS. This is measured message exchange on one workstation, not physical cooperation or production validation.

Use the standalone host profile on trusted Linux hosts only. Stop any previous manually launched am-mission, am-coordination, am-evaluation containers first to prevent duplicate identities. Do not combine the two Compose files.

```bash
docker compose -f compose.agents.host.yaml build mission
docker compose -f compose.agents.host.yaml up -d mission coordination evaluation
docker compose -f compose.agents.host.yaml run --rm probe
docker compose -f compose.agents.host.yaml stop coordination
docker compose -f compose.agents.host.yaml run --rm probe --scenario /workspace/pilots/industrial_inspection.json --expect-no-coordinator
docker compose -f compose.agents.host.yaml start coordination
docker compose -f compose.agents.host.yaml run --rm probe
docker compose -f compose.agents.host.yaml down
```

The host profile preserves the validated network/IPC/domain settings; the Compose wrapper and missing-coordinator experiment still require execution. Set AUTONOMYMIND_AGENT_DOMAIN to a free DDS domain if needed. Host networking and IPC reduce isolation and can expose DDS to other reachable hosts; a domain ID is not a security boundary. Multi-host DDS configuration and authenticated transport remain pending.
