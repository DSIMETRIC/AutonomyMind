#pragma once
#include "autonomymind/trajectory_generator.hpp"

namespace autonomymind {

struct RtaConstraints {
  double min_separation_m{};
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
};

struct RtaRequest {
  SmoothTrajectoryResult trajectory{};
  double sample_dt_s{};
  double own_x_m{}, own_y_m{}, own_z_m{};
  double own_vx_mps{}, own_vy_mps{}, own_vz_mps{};
  double threat_rel_x_m{}, threat_rel_y_m{}, threat_rel_z_m{};
  double relative_vx_mps{}, relative_vy_mps{}, relative_vz_mps{};
  RtaConstraints constraints{};
};

struct RtaResult {
  bool valid{false};
  bool traffic_safe{false};
  bool terrain_safe{false};
  bool geofence_safe{false};
  bool envelope_safe{false};
  bool safe{false};
  double predicted_min_separation_m{};
  double minimum_terrain_clearance_m{};
};

class FullRtaChecker {
 public:
  RtaResult evaluate(const RtaRequest& request) const;
};

} // namespace autonomymind
