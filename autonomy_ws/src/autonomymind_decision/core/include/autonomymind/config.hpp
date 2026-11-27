#pragma once
#include <array>
#include <string>

namespace autonomymind {

struct CoreConfig {
  double unit_interval_min{};
  double unit_interval_max{};
  double numeric_epsilon{};
  double pi_rad{};
  double heading_half_range_deg{};
  double heading_full_range_deg{};
  double advisory_authority_pct{};

  double alignment_calibration_s{};
  int alignment_min_samples{};

  int nav_minimum_operational_sources{};
  int nav_sources_required{};
  bool nav_source_b_enabled{};
  double nav_filter_tau_s{};
  double nav_verify_enter_m{};
  double nav_verify_exit_m{};
  double nav_verify_enter_s{};
  double nav_verify_exit_s{};
  double nav_integrity_full_scale_m{};
  double nav_integrity_degrade_min{};
  double nav_integrity_hold_min{};
  // Integrity credited to a single source without independent cross-check.
  double nav_single_source_integrity{};

  // Optional advisory ML ranker. It may rank already-safe avoidance candidates,
  // but it never bypasses the deterministic Behavior Trees or Full RTA gates.
  bool ml_avoidance_ranker_enabled{};
  double ml_minimum_confidence{};
  std::string ml_avoidance_ranker_model_path;
  std::string ml_model_name;
  std::string ml_model_version;
  std::string ml_feature_schema;
  std::string ml_model_sha256;
  std::array<double, 8> ml_weights{};
  std::array<double, 3> ml_maneuver_bias{};

  double rejoin_enter_m{};
  double rejoin_exit_m{};
  double rejoin_heading_hold_deg{};

  double high_dynamics_speed_mps{};
  double high_dynamics_accel_mps2{};
  double high_dynamics_jerk_mps3{};
  double high_dynamics_yaw_rate_dps{};
  double vertical_transition_rate_mps{};

  double traffic_min_separation_m{};
  double traffic_prediction_horizon_s{};
  double traffic_prediction_dt_s{};
  // Candidate trajectory shaping.
  double traffic_lateral_avoid_speed_mps{};  // Legacy compatibility; not used by coordinated-turn generation.
  double traffic_climb_avoid_speed_mps{};
  double traffic_maneuver_duration_s{};
  double traffic_turn_angle_deg{};
  double traffic_min_turn_speed_mps{};
  double gravity_mps2{};

  // Full RTA baseline constraints. These values are configuration-driven so
  // the safety checker never relies on hidden literals in C++.
  double rta_terrain_elevation_m{};
  double rta_min_terrain_clearance_m{};
  double rta_geofence_min_x_m{};
  double rta_geofence_max_x_m{};
  double rta_geofence_min_y_m{};
  double rta_geofence_max_y_m{};
  double rta_geofence_min_z_m{};
  double rta_geofence_max_z_m{};
  double rta_max_speed_mps{};
  double rta_max_vertical_speed_mps{};
  double rta_max_accel_mps2{};
  double rta_max_jerk_mps3{};
  double rta_max_yaw_rate_dps{};
  double rta_max_bank_angle_deg{};

  // Legacy names retained for profile compatibility while the dashboard and
  // deployment tooling migrate to the explicit Full-RTA envelope fields.
  double traffic_max_lateral_accel_mps2{};
  double traffic_max_vertical_accel_mps2{};
  double traffic_max_avoid_jerk_mps3{};

  std::string profile_name;
  std::string sha256;
  std::string source_path;
  std::string baseline_path;

  static CoreConfig load_strict(const std::string& path);
  void validate() const;
};

} // namespace autonomymind
