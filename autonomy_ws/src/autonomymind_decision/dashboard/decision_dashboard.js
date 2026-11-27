"use strict";

function passFail(value) {
  return value ? "PASS" : "FAIL";
}

function buildDecisionViewModel(data) {
  if (!data || !Array.isArray(data.candidates)) {
    throw new Error("invalid Decision dashboard evidence");
  }

  const candidates = data.candidates.map((candidate, index) => ({
    name: candidate.name,
    selected: index === data.selected_index,
    dynamic: passFail(candidate.dynamically_feasible),
    traffic: passFail(candidate.traffic_safe),
    terrain: passFail(candidate.terrain_safe),
    geofence: passFail(candidate.geofence_safe),
    envelope: passFail(candidate.envelope_safe),
    fullRta: passFail(candidate.rta_safe),
    minSeparationM: candidate.predicted_min_separation_m,
    terrainClearanceM: candidate.minimum_terrain_clearance_m,
    peakSpeedMps: candidate.peak_speed_mps,
    peakVerticalSpeedMps: candidate.peak_vertical_speed_mps,
    peakAccelMps2: candidate.peak_accel_mps2,
    peakJerkMps3: candidate.peak_jerk_mps3,
    peakYawRateDps: candidate.peak_yaw_rate_dps,
    peakBankAngleDeg: candidate.peak_bank_angle_deg,
  }));

  const ml = data.ml || {};
  return {
    decision: data.decision,
    samples: data.samples,
    avoidCount: data.avoid_count,
    rejoinCount: data.rejoin_count,
    fullRtaSamples: data.selected_full_rta_samples,
    ml: {
      enabled: Boolean(ml.enabled),
      inferenceValid: Boolean(ml.inference_valid),
      usedForSelection: Boolean(ml.used_for_selection),
      proposalAccepted: Boolean(ml.proposal_accepted),
      proposedCandidate: ml.proposed_candidate ?? -1,
      selectedCandidate: ml.selected_candidate ?? -1,
      confidence: ml.confidence ?? 0,
      modelName: ml.model_name || "",
      modelVersion: ml.model_version || "",
      modelSha256: ml.model_sha256 || "",
      featureSchema: ml.feature_schema || "",
      fallbackReason: ml.fallback_reason || "",
      candidateScores: Array.isArray(ml.candidate_scores) ? ml.candidate_scores : [],
      features: Array.isArray(ml.features) ? ml.features : [],
    },
    candidates,
  };
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { buildDecisionViewModel };
}
