#include "autonomymind/behavior_tree.hpp"
#include "autonomymind/config.hpp"

#include <iostream>

using namespace autonomymind;

CollisionAvoidanceRequest requestFromConfig(const CoreConfig& cfg) {
  CollisionAvoidanceRequest r;
  r.traffic_conflict = true;
  r.rel_x_m = 40.0;
  r.rel_y_m = 10.0;
  r.relative_vx_mps = -10.0;
  r.min_separation_m = cfg.traffic_min_separation_m;
  r.prediction_horizon_s = cfg.traffic_prediction_horizon_s;
  r.prediction_dt_s = cfg.traffic_prediction_dt_s;
  r.own_z_m = 100.0;
  r.own_vx_mps = 5.0;
  r.maneuver_duration_s = cfg.traffic_maneuver_duration_s;
  r.turn_angle_deg = cfg.traffic_turn_angle_deg;
  r.min_turn_speed_mps = cfg.traffic_min_turn_speed_mps;
  r.climb_speed_mps = cfg.traffic_climb_avoid_speed_mps;
  r.gravity_mps2 = cfg.gravity_mps2;
  r.terrain_elevation_m = cfg.rta_terrain_elevation_m;
  r.min_terrain_clearance_m = cfg.rta_min_terrain_clearance_m;
  r.geofence_min_x_m = cfg.rta_geofence_min_x_m;
  r.geofence_max_x_m = cfg.rta_geofence_max_x_m;
  r.geofence_min_y_m = cfg.rta_geofence_min_y_m;
  r.geofence_max_y_m = cfg.rta_geofence_max_y_m;
  r.geofence_min_z_m = cfg.rta_geofence_min_z_m;
  r.geofence_max_z_m = cfg.rta_geofence_max_z_m;
  r.max_speed_mps = cfg.rta_max_speed_mps;
  r.max_vertical_speed_mps = cfg.rta_max_vertical_speed_mps;
  r.max_accel_mps2 = cfg.rta_max_accel_mps2;
  r.max_jerk_mps3 = cfg.rta_max_jerk_mps3;
  r.max_yaw_rate_dps = cfg.rta_max_yaw_rate_dps;
  r.max_bank_angle_deg = cfg.rta_max_bank_angle_deg;
  r.ml_ranker_enabled = cfg.ml_avoidance_ranker_enabled;
  r.ml_minimum_confidence = cfg.ml_minimum_confidence;
  r.ml_weights = cfg.ml_weights;
  r.ml_maneuver_bias = cfg.ml_maneuver_bias;
  r.ml_model_name = cfg.ml_model_name;
  r.ml_model_version = cfg.ml_model_version;
  r.ml_model_sha256 = cfg.ml_model_sha256;
  r.ml_feature_schema = cfg.ml_feature_schema;
  return r;
}

int main() {
  const auto cfg = CoreConfig::load_strict(FM_TEST_CONFIG);
  if (!cfg.ml_avoidance_ranker_enabled || cfg.ml_model_sha256.size() != 64u) {
    return 1;
  }

  const CollisionAvoidanceTree tree;

  // Normal inference must preserve model provenance and can select only a
  // candidate that already passed Full RTA.
  {
    auto r = requestFromConfig(cfg);
    const auto out = tree.tick(r);
    if (out.status != BehaviorStatus::SUCCESS ||
        !out.ml_trace.enabled ||
        !out.ml_trace.inference_valid ||
        out.ml_trace.model_sha256 != cfg.ml_model_sha256 ||
        out.selected_index < 0 ||
        !out.candidates[static_cast<std::size_t>(out.selected_index)].rta_safe) {
      return 2;
    }
    if (out.ml_trace.used_for_selection &&
        out.ml_trace.selected_candidate != out.ml_trace.proposed_candidate) {
      return 3;
    }
  }

  // Prove that ML can change preference among candidates that are all already
  // safe, without changing any safety gate. A distant static threat makes the
  // candidates equivalent enough for a maneuver bias to drive the ranking.
  {
    auto deterministic_request = requestFromConfig(cfg);
    deterministic_request.rel_x_m = 100.0;
    deterministic_request.rel_y_m = 100.0;
    deterministic_request.relative_vx_mps = 0.0;
    deterministic_request.ml_ranker_enabled = false;
    const auto deterministic = tree.tick(deterministic_request);
    if (deterministic.status != BehaviorStatus::SUCCESS) return 4;

    int alternate = -1;
    for (int index = 0; index < 3; ++index) {
      if (index != deterministic.selected_index &&
          deterministic.candidates[static_cast<std::size_t>(index)].rta_safe) {
        alternate = index;
        break;
      }
    }
    if (alternate < 0) return 5;

    auto learned_request = deterministic_request;
    learned_request.ml_ranker_enabled = true;
    learned_request.ml_weights.fill(0.0);
    learned_request.ml_maneuver_bias.fill(0.0);
    learned_request.ml_maneuver_bias[static_cast<std::size_t>(alternate)] = 10.0;
    learned_request.ml_minimum_confidence = 0.60;
    const auto learned = tree.tick(learned_request);
    if (!learned.ml_trace.used_for_selection ||
        learned.selected_index != alternate ||
        !learned.candidates[static_cast<std::size_t>(alternate)].rta_safe) {
      return 6;
    }
  }

  // Make RIGHT violate the geofence, then give it an overwhelming model bias.
  // The unsafe candidate must still be excluded before ML arbitration.
  {
    auto r = requestFromConfig(cfg);
    r.geofence_min_y_m = 0.0;
    r.ml_maneuver_bias[1] = 1000000.0;
    const auto out = tree.tick(r);
    if (out.candidates[1].rta_safe || out.selected_index == 1 ||
        out.ml_trace.proposed_candidate == 1) {
      return 7;
    }
  }

  // Missing model provenance disables ML selection even when the feature is
  // nominally enabled; deterministic arbitration remains available.
  {
    auto r = requestFromConfig(cfg);
    r.ml_model_sha256.clear();
    const auto out = tree.tick(r);
    if (out.status != BehaviorStatus::SUCCESS ||
        out.ml_trace.used_for_selection ||
        out.ml_trace.inference_valid ||
        out.ml_trace.fallback_reason !=
            "ML provenance or feature schema invalid") {
      return 8;
    }
  }

  // A confidence threshold above the possible [0,1] range is not a valid
  // deployment config, but at request level it demonstrates deterministic
  // fallback without allowing ML to become the safety authority.
  {
    auto r = requestFromConfig(cfg);
    r.ml_minimum_confidence = 1.1;
    const auto out = tree.tick(r);
    if (out.status != BehaviorStatus::SUCCESS ||
        out.ml_trace.used_for_selection ||
        out.ml_trace.fallback_reason != "ML confidence below threshold") {
      return 9;
    }
  }

  std::cout << "PASS: auditable ML ranks only Full-RTA-safe candidates with deterministic fallback\n";
  return 0;
}
