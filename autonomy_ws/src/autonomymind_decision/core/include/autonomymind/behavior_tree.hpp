#pragma once
#include "autonomymind/types.hpp"
#include "autonomymind/trajectory_generator.hpp"
#include "autonomymind/rta.hpp"
#include <array>
#include <cstdint>
#include <string>

namespace autonomymind {

enum class BehaviorStatus : std::int32_t { FAILURE=0, SUCCESS=1, RUNNING=2 };
enum class AvoidanceManeuver : std::int32_t { LEFT=0, RIGHT=1, CLIMB=2 };

struct AvoidanceCandidate {
  AvoidanceManeuver maneuver{AvoidanceManeuver::LEFT};
  Decision decision{Decision::AVOID_LEFT};
  Guidance guidance{Guidance::TURN_LEFT};
  double predicted_min_separation_m{};
  double minimum_terrain_clearance_m{};
  bool dynamically_feasible{false};
  bool traffic_safe{false};
  bool terrain_safe{false};
  bool geofence_safe{false};
  bool envelope_safe{false};
  bool rta_safe{false};
  double peak_speed_mps{};
  double peak_vertical_speed_mps{};
  double peak_accel_mps2{};
  double peak_jerk_mps3{};
  double peak_yaw_rate_dps{};
  double peak_bank_angle_deg{};
  int point_count{};
  double trajectory_x_m[FM_TRAJECTORY_POINTS]{};
  double trajectory_y_m[FM_TRAJECTORY_POINTS]{};
  double trajectory_z_m[FM_TRAJECTORY_POINTS]{};
};

struct CollisionAvoidanceRequest {
  bool traffic_conflict{false};
  double rel_x_m{}, rel_y_m{}, rel_z_m{};
  double relative_vx_mps{}, relative_vy_mps{}, relative_vz_mps{};
  double min_separation_m{};
  double prediction_horizon_s{};
  double prediction_dt_s{};
  double own_x_m{}, own_y_m{}, own_z_m{};
  double own_vx_mps{}, own_vy_mps{}, own_vz_mps{};
  double maneuver_duration_s{};
  double turn_angle_deg{};
  double min_turn_speed_mps{};
  double climb_speed_mps{};
  double gravity_mps2{};
  double terrain_elevation_m{};
  double min_terrain_clearance_m{};
  double geofence_min_x_m{}, geofence_max_x_m{};
  double geofence_min_y_m{}, geofence_max_y_m{};
  double geofence_min_z_m{}, geofence_max_z_m{};
  double max_speed_mps{};
  double max_vertical_speed_mps{};
  double max_accel_mps2{};
  double max_jerk_mps3{};
  double max_yaw_rate_dps{};
  double max_bank_angle_deg{};
  bool ml_ranker_enabled{false};
  double ml_minimum_confidence{};
  std::array<double, 8> ml_weights{};
  std::array<double, 3> ml_maneuver_bias{};
  std::string ml_model_name;
  std::string ml_model_version;
  std::string ml_model_sha256;
  std::string ml_feature_schema;
};

struct CollisionAvoidanceResponse {
  BehaviorStatus status{BehaviorStatus::FAILURE};
  Decision candidate_decision{Decision::CONTINUE_MISSION};
  Guidance candidate_guidance{Guidance::FOLLOW_MISSION};
  double selected_predicted_min_separation_m{};
  int safe_candidate_count{};
  int selected_index{-1};
  std::array<AvoidanceCandidate,3> candidates{};
  MlTrace ml_trace{};
};

class CollisionAvoidanceTree {
 public:
  CollisionAvoidanceResponse tick(const CollisionAvoidanceRequest& request) const;
 private:
  static AvoidanceCandidate evaluate(const CollisionAvoidanceRequest& request,
                                     AvoidanceManeuver maneuver);
};

} // namespace autonomymind
