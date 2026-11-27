#include "autonomymind/behavior_tree.hpp"

#include <algorithm>
#include <cmath>

namespace autonomymind {

AvoidanceCandidate CollisionAvoidanceTree::evaluate(
    const CollisionAvoidanceRequest& request,
    AvoidanceManeuver maneuver) {
  AvoidanceCandidate candidate;
  candidate.maneuver = maneuver;

  // Map each candidate maneuver to the operational Decision/Guidance pair.
  // The behavior tree proposes candidates; Full RTA decides whether a proposed
  // trajectory is admissible.
  if (maneuver == AvoidanceManeuver::LEFT) {
    candidate.decision = Decision::AVOID_LEFT;
    candidate.guidance = Guidance::TURN_LEFT;
  } else if (maneuver == AvoidanceManeuver::RIGHT) {
    candidate.decision = Decision::AVOID_RIGHT;
    candidate.guidance = Guidance::TURN_RIGHT;
  } else {
    candidate.decision = Decision::AVOID_CLIMB;
    candidate.guidance = Guidance::CLIMB;
  }

  SmoothTrajectoryRequest trajectory_request;
  trajectory_request.maneuver =
      maneuver == AvoidanceManeuver::LEFT
          ? SmoothManeuver::LEFT
          : (maneuver == AvoidanceManeuver::RIGHT
                 ? SmoothManeuver::RIGHT
                 : SmoothManeuver::CLIMB);

  trajectory_request.own_vx_mps = request.own_vx_mps;
  trajectory_request.own_vy_mps = request.own_vy_mps;
  trajectory_request.own_vz_mps = request.own_vz_mps;
  trajectory_request.horizon_s = request.prediction_horizon_s;
  trajectory_request.sample_dt_s = request.prediction_dt_s;
  trajectory_request.maneuver_duration_s = request.maneuver_duration_s;
  trajectory_request.turn_angle_deg = request.turn_angle_deg;
  trajectory_request.min_turn_speed_mps = request.min_turn_speed_mps;
  trajectory_request.climb_speed_mps = request.climb_speed_mps;
  trajectory_request.gravity_mps2 = request.gravity_mps2;

  // Generate the physical candidate before performing any assurance check.
  // LEFT/RIGHT use the coordinated-turn kinematic model; CLIMB uses the
  // smooth vertical-speed profile.
  const SmoothTrajectoryGenerator trajectory_generator;
  const auto trajectory =
      trajectory_generator.evaluate(trajectory_request);

  candidate.dynamically_feasible =
      trajectory.valid && trajectory.dynamically_feasible;
  candidate.peak_speed_mps = trajectory.peak_speed_mps;
  candidate.peak_vertical_speed_mps =
      trajectory.peak_vertical_speed_mps;
  candidate.peak_accel_mps2 = trajectory.peak_accel_mps2;
  candidate.peak_jerk_mps3 = trajectory.peak_jerk_mps3;
  candidate.peak_yaw_rate_dps = trajectory.peak_yaw_rate_dps;
  candidate.peak_bank_angle_deg = trajectory.peak_bank_angle_deg;

  candidate.point_count = std::max(
      0,
      std::min(FM_TRAJECTORY_POINTS, trajectory.point_count));

  for (int index = 0; index < candidate.point_count; ++index) {
    candidate.trajectory_x_m[index] =
        trajectory.trajectory_x_m[index];
    candidate.trajectory_y_m[index] =
        trajectory.trajectory_y_m[index];
    candidate.trajectory_z_m[index] =
        trajectory.trajectory_z_m[index];
  }

  // Build the complete RTA request explicitly. Named assignments are used on
  // purpose: positional aggregate initialization is unsafe here because adding
  // a new constraint could silently shift unrelated safety parameters.
  RtaRequest rta_request;
  rta_request.trajectory = trajectory;
  rta_request.sample_dt_s = request.prediction_dt_s;

  rta_request.own_x_m = request.own_x_m;
  rta_request.own_y_m = request.own_y_m;
  rta_request.own_z_m = request.own_z_m;
  rta_request.own_vx_mps = request.own_vx_mps;
  rta_request.own_vy_mps = request.own_vy_mps;
  rta_request.own_vz_mps = request.own_vz_mps;

  rta_request.threat_rel_x_m = request.rel_x_m;
  rta_request.threat_rel_y_m = request.rel_y_m;
  rta_request.threat_rel_z_m = request.rel_z_m;
  rta_request.relative_vx_mps = request.relative_vx_mps;
  rta_request.relative_vy_mps = request.relative_vy_mps;
  rta_request.relative_vz_mps = request.relative_vz_mps;

  rta_request.constraints.min_separation_m =
      request.min_separation_m;
  rta_request.constraints.terrain_elevation_m =
      request.terrain_elevation_m;
  rta_request.constraints.min_terrain_clearance_m =
      request.min_terrain_clearance_m;

  rta_request.constraints.geofence_min_x_m =
      request.geofence_min_x_m;
  rta_request.constraints.geofence_max_x_m =
      request.geofence_max_x_m;
  rta_request.constraints.geofence_min_y_m =
      request.geofence_min_y_m;
  rta_request.constraints.geofence_max_y_m =
      request.geofence_max_y_m;
  rta_request.constraints.geofence_min_z_m =
      request.geofence_min_z_m;
  rta_request.constraints.geofence_max_z_m =
      request.geofence_max_z_m;

  rta_request.constraints.max_speed_mps =
      request.max_speed_mps;
  rta_request.constraints.max_vertical_speed_mps =
      request.max_vertical_speed_mps;
  rta_request.constraints.max_accel_mps2 =
      request.max_accel_mps2;
  rta_request.constraints.max_jerk_mps3 =
      request.max_jerk_mps3;
  rta_request.constraints.max_yaw_rate_dps =
      request.max_yaw_rate_dps;
  rta_request.constraints.max_bank_angle_deg =
      request.max_bank_angle_deg;

  // Full RTA is the assurance authority for a candidate. It evaluates traffic
  // separation, terrain clearance, geofence containment, and flight envelope.
  const FullRtaChecker rta_checker;
  const auto assurance = rta_checker.evaluate(rta_request);

  candidate.predicted_min_separation_m =
      assurance.predicted_min_separation_m;
  candidate.minimum_terrain_clearance_m =
      assurance.minimum_terrain_clearance_m;
  candidate.traffic_safe = assurance.traffic_safe;
  candidate.terrain_safe = assurance.terrain_safe;
  candidate.geofence_safe = assurance.geofence_safe;
  candidate.envelope_safe = assurance.envelope_safe;

  // Fail closed: a candidate cannot enter arbitration unless both trajectory
  // generation and every Full-RTA gate have passed.
  candidate.rta_safe =
      candidate.dynamically_feasible && assurance.safe;

  return candidate;
}

CollisionAvoidanceResponse CollisionAvoidanceTree::tick(
    const CollisionAvoidanceRequest& request) const {
  CollisionAvoidanceResponse output;

  // Reject invalid collision-assessment inputs before candidate generation.
  // Returning FAILURE causes the caller to use its fail-safe behavior.
  const bool invalid_request =
      !request.traffic_conflict ||
      request.min_separation_m <= 0.0 ||
      request.prediction_horizon_s <= 0.0 ||
      request.prediction_dt_s <= 0.0 ||
      request.prediction_dt_s > request.prediction_horizon_s ||
      request.maneuver_duration_s <= 0.0 ||
      request.maneuver_duration_s > request.prediction_horizon_s ||
      request.turn_angle_deg <= 0.0 ||
      request.min_turn_speed_mps <= 0.0 ||
      request.climb_speed_mps <= 0.0 ||
      request.gravity_mps2 <= 0.0;

  if (invalid_request) {
    return output;
  }

  // Generate and assure all supported avoidance alternatives. Candidate order
  // is deterministic so equal-score cases are reproducible.
  output.candidates[0] =
      evaluate(request, AvoidanceManeuver::LEFT);
  output.candidates[1] =
      evaluate(request, AvoidanceManeuver::RIGHT);
  output.candidates[2] =
      evaluate(request, AvoidanceManeuver::CLIMB);

  int deterministic_selected_index = -1;
  int ml_selected_index = -1;
  double best_ml_score = 0.0;
  double second_ml_score = 0.0;
  bool have_best_ml_score = false;
  bool have_second_ml_score = false;

  output.ml_trace.enabled = request.ml_ranker_enabled;
  output.ml_trace.model_name = request.ml_model_name;
  output.ml_trace.model_version = request.ml_model_version;
  output.ml_trace.model_sha256 = request.ml_model_sha256;
  output.ml_trace.feature_schema = request.ml_feature_schema;

  // Enabled inference is accepted only when the exact model identity and
  // feature contract are present. Missing provenance falls back deterministically.
  const bool ml_ready =
      request.ml_ranker_enabled &&
      !request.ml_model_name.empty() &&
      !request.ml_model_version.empty() &&
      request.ml_model_sha256.size() == 64u &&
      request.ml_feature_schema == "rta-safe-v1";
  if (request.ml_ranker_enabled && !ml_ready) {
    output.ml_trace.fallback_reason = "ML provenance or feature schema invalid";
  }

  auto clamp01 = [](double value) {
    return std::max(0.0, std::min(1.0, value));
  };

  for (int index = 0; index < 3; ++index) {
    const auto& candidate =
        output.candidates[static_cast<std::size_t>(index)];

    // Unsafe candidates never enter either deterministic or ML arbitration.
    if (!candidate.rta_safe) {
      continue;
    }

    ++output.safe_candidate_count;

    if (deterministic_selected_index < 0 ||
        candidate.predicted_min_separation_m >
            output.candidates[
                static_cast<std::size_t>(deterministic_selected_index)]
                .predicted_min_separation_m) {
      deterministic_selected_index = index;
    }

    if (!ml_ready) {
      continue;
    }

    // The ranker sees only post-RTA evidence. Every feature is normalized by a
    // deterministic safety limit so the model cannot alter the meaning of any
    // gate or introduce hidden unit conversions.
    const std::array<double, 8> features = {
        candidate.predicted_min_separation_m /
            std::max(request.min_separation_m, 1.0),
        candidate.minimum_terrain_clearance_m /
            std::max(request.min_terrain_clearance_m, 1.0),
        clamp01((request.max_speed_mps - candidate.peak_speed_mps) /
                std::max(request.max_speed_mps, 1.0)),
        clamp01((request.max_vertical_speed_mps -
                 candidate.peak_vertical_speed_mps) /
                std::max(request.max_vertical_speed_mps, 1.0)),
        clamp01((request.max_accel_mps2 - candidate.peak_accel_mps2) /
                std::max(request.max_accel_mps2, 1.0)),
        clamp01((request.max_jerk_mps3 - candidate.peak_jerk_mps3) /
                std::max(request.max_jerk_mps3, 1.0)),
        clamp01((request.max_yaw_rate_dps - candidate.peak_yaw_rate_dps) /
                std::max(request.max_yaw_rate_dps, 1.0)),
        clamp01((request.max_bank_angle_deg -
                 candidate.peak_bank_angle_deg) /
                std::max(request.max_bank_angle_deg, 1.0)),
    };

    for (std::size_t feature = 0; feature < features.size(); ++feature) {
      output.ml_trace.feature_value[index][feature] = features[feature];
    }

    double score =
        request.ml_maneuver_bias[static_cast<std::size_t>(index)];
    for (std::size_t feature = 0; feature < features.size(); ++feature) {
      score += request.ml_weights[feature] * features[feature];
    }
    output.ml_trace.candidate_score[index] = score;

    if (!std::isfinite(score)) {
      output.ml_trace.fallback_reason = "non-finite ML score";
      continue;
    }

    if (!have_best_ml_score || score > best_ml_score) {
      if (have_best_ml_score) {
        second_ml_score = best_ml_score;
        have_second_ml_score = true;
      }
      best_ml_score = score;
      have_best_ml_score = true;
      ml_selected_index = index;
    } else if (!have_second_ml_score || score > second_ml_score) {
      second_ml_score = score;
      have_second_ml_score = true;
    }
  }

  int selected_index = deterministic_selected_index;

  if (ml_ready && have_best_ml_score &&
      ml_selected_index >= 0) {
    output.ml_trace.inference_valid = true;
    output.ml_trace.proposed_candidate = ml_selected_index;

    // Confidence is the logistic transform of the top-two score margin. With
    // one safe candidate there is no ranking ambiguity.
    output.ml_trace.confidence =
        have_second_ml_score
            ? 1.0 / (1.0 + std::exp(-(best_ml_score - second_ml_score)))
            : 1.0;

    if (output.ml_trace.confidence >= request.ml_minimum_confidence) {
      selected_index = ml_selected_index;
      output.ml_trace.used_for_selection = true;
      output.ml_trace.proposal_accepted = true;
    } else {
      output.ml_trace.fallback_reason = "ML confidence below threshold";
    }
  } else if (!request.ml_ranker_enabled) {
    output.ml_trace.fallback_reason = "ML ranker disabled";
  } else if (output.ml_trace.fallback_reason.empty()) {
    output.ml_trace.fallback_reason = "ML inference unavailable";
  }

  output.ml_trace.selected_candidate = selected_index;

  // No Full-RTA-safe maneuver exists. FAILURE deliberately leaves the final
  // fail-safe decision (normally HOLD) to the mission decision authority.
  if (selected_index < 0) {
    return output;
  }

  const auto& selected =
      output.candidates[static_cast<std::size_t>(selected_index)];

  output.status = BehaviorStatus::SUCCESS;
  output.selected_index = selected_index;
  output.candidate_decision = selected.decision;
  output.candidate_guidance = selected.guidance;
  output.selected_predicted_min_separation_m =
      selected.predicted_min_separation_m;

  return output;
}

}  // namespace autonomymind
