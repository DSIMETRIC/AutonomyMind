#pragma once
#include "autonomymind/generated_limits.h"
#include <cstdint>

namespace autonomymind {

enum class SmoothManeuver : std::int32_t { LEFT=0, RIGHT=1, CLIMB=2 };

struct SmoothTrajectoryRequest {
  SmoothManeuver maneuver{SmoothManeuver::LEFT};
  double own_vx_mps{}, own_vy_mps{}, own_vz_mps{};
  double horizon_s{};
  double sample_dt_s{};
  double maneuver_duration_s{};
  double turn_angle_deg{};
  double min_turn_speed_mps{};
  double climb_speed_mps{};
  double gravity_mps2{};
};

struct SmoothTrajectoryResult {
  bool valid{false};
  bool dynamically_feasible{false};
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

class SmoothTrajectoryGenerator {
 public:
  SmoothTrajectoryResult evaluate(const SmoothTrajectoryRequest& request) const;
};

} // namespace autonomymind
