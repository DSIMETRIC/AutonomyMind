# Advisory model backends

The model registry defines an interchangeable inference interface for ranking supplied candidate IDs, with model identity, version, declared artifact digest and input digest. A deterministic ReferenceRanker is the baseline. A backend may only return a permutation of the candidate set and a bounded explanation; unknown output fields or invented candidates are rejected. The result always requires independent safety validation and never authorizes motion.

This is a Python reference interface, not actual ONNX, TensorRT or LLM integration. Model digests are declared metadata, not verified against artifacts yet. Backends are trusted code; timeouts, process isolation, artifact verification, resource budgets and model release approval remain pending. The DDS worker does not call this interface yet. Future coordination and planning agents can rank independently generated feasible proposals; deterministic RTA remains outside the model.
