# DDS advisory trajectory reservations

A separate ROS 2 DDS node exposes reservation.submit and reservation.query with correlated replies and heartbeat discovery. One synthetic fleet-batch producer submits all member proposals; this is not yet independent per-vehicle negotiation or distributed consensus. Membership and search settings are read from scenario configuration. The service uses the existing deterministic coordinator and bounded DDS envelope receiver, rejects stale revisions and incomplete batches, publishes no actuator commands and writes an expiring advisory state. The probe checks a priority revision, pairwise conflicts (same geometric checker, not independent RTA), stale revision rejection, incomplete batch rejection and expiry.

```bash
docker compose run --build --rm spatiotemporal-pilot
docker compose -f compose.agents.host.yaml build mission
docker compose -f compose.agents.host.yaml -f compose.agents.reservations.yaml up -d --force-recreate reservations
docker compose -f compose.agents.host.yaml -f compose.agents.reservations.yaml run --rm reservation-probe
```

Reports: semantic_data/outbox/reservation_probe.json and reservation_state.json. DDS uses local trusted host networking and IPC, no authentication, persistence, real vehicle state validation, distributed clocks or failover. Wall-clock time is used; clock rollback requires restart. Lease expiry removes the advisory, not physical occupancy. A rejected malformed or incomplete batch does not replace an existing plan. Revision numbers reset on restart. DDS batch capped at eight participants and 32 points per participant; underlying offline coordinator supports up to 32. No live Gazebo integration or C++ RTA assessment. Offline and DDS tests remain unexecuted at publication.
