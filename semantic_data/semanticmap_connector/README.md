# SemanticMap Connector

This directory defines the public integration boundary between AutonomyMind and semantic annotation / mapping systems.

The connector must not depend on private implementation details of any annotation platform.

## Responsibilities

- submit annotation requests;
- receive reviewed semantic geometry;
- exchange dataset and model provenance;
- synchronize evaluation metadata;
- import/export semantic map features;
- preserve coordinate-frame and timestamp metadata.

## Example annotation payload

```json
{
  "schema_version": "1.0",
  "frame_id": "camera_front",
  "timestamp": 0.0,
  "label": "crosswalk",
  "geometry_type": "polygon",
  "geometry": [],
  "source": "agent_proposal",
  "model_version": "example",
  "review_status": "pending"
}
```

The example is intentionally generic and contains no credentials, private endpoints, or deployment-specific data.
