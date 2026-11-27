# Local LLM and VLM agents — experimental adapters

Optional Ollama-compatible loopback HTTP backend for coordination ranking and offline visual scene interpretation. No model is downloaded or bundled. The operator must install a compatible local model server, select models whose licenses permit the intended use, and verify artifacts. The LLM model digest is declared metadata, not verified by the adapter. The backend does not expose tools or actuator APIs.

LLM ranking uses the existing deterministic eligibility filter, mission dependencies and leases. It is an alternative to the ONNX profile, not a simultaneous second authority. Invalid responses cause no offer for the affected task. VLM descriptions contain unverified semantic labels, never metric geometry, and are published on the reference DDS event bus; no World Model consumer is implemented yet.

## Configuration

Create semantic_data/outbox/reasoning/llm.json with fields id, version, url (http://127.0.0.1:11434), model (installed model name), declared_sha256 (actual operator-recorded 64-character lowercase hex digest), timeout_s. Create vlm.json with url, model and timeout_s; select an installed image-capable model. Put an authorized JPEG at semantic_data/outbox/vision_input/frame.jpg. Do not commit images, model weights, credentials or restricted datasets.

```bash
docker compose -f compose.agents.host.yaml build mission
docker compose -f compose.agents.host.yaml -f compose.agents.reasoning.yaml up -d mission coordination evaluation
docker compose -f compose.agents.host.yaml -f compose.agents.reasoning.yaml run --rm vision
```

The existing synthetic replay probe has a 20-second result deadline and expects a particular assignment sequence. LLM inference may choose a different valid assignment or exceed that deadline, so it is not an acceptance test for the LLM backend. Use the ONNX/reference profile for the previous regression. A future benchmark must compare feasible assignments and measured latency rather than a fixed outcome.

Known limitations: synchronous inference can block DDS heartbeats; HTTP timeout is not a hard end-to-end deadline. No process isolation, model provenance verification, live visual stream, geometric fusion, tool orchestration, learned policy validation or physical control. Host network is trusted development only. The VLM one-shot publisher verifies DDS acknowledgment, not application consumption. Mock tests do not establish actual model inference. Do not use for live vehicle decisions.
