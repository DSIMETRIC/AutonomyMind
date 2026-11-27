# Local learned-ranking reference for Coordination Agent

Optional CPU ONNX inference ranks only fresh, available, capability-compatible members for ready tasks. The deterministic coordinator still controls dependencies, uniqueness, expiry and assignment. Model failures produce no offer for that task. Model output cannot authorize movement. Artifacts are loaded from verified bytes and recorded with input digest in mission events.

The example is a linear regression trained on synthetic utility labels, exported to ONNX. It is not a validated learned fleet policy, LLM or VLM. It uses distance, telemetry age, uncertainty and task priority. No Waymo data or third-party weights are included. CPU only; GPU, TensorRT, isolated inference deadlines and real training/evaluation remain future work.

On trusted Linux, stop any old manual agents and previous conflicting DDS stacks before running:

```bash
docker compose -f compose.agents.host.yaml build mission
docker compose -f compose.agents.host.yaml -f compose.agents.ai.yaml run --build --rm train-ranker
docker compose -f compose.agents.host.yaml -f compose.agents.ai.yaml up -d --build mission coordination evaluation
docker compose -f compose.agents.host.yaml -f compose.agents.ai.yaml run --rm probe
```

Inspect semantic_data/outbox/dds_pilot.json for model_advice events, model identity and SHA-256. The probe checks the existing synthetic mission acceptance metrics; it is not a physical autonomy test. Unit tests cover eligibility and rejection. Docker inference and DDS integration must be executed before reporting success. No autonomous vehicle actuation or safety certification.
