#include "autonomymind/rta.hpp"
#include "autonomymind/trajectory_generator.hpp"

#include <iostream>

namespace {

autonomymind::SmoothTrajectoryResult trajectory(
    autonomymind::SmoothManeuver maneuver =
        autonomymind::SmoothManeuver::RIGHT) {
  autonomymind::SmoothTrajectoryRequest request;
  request.maneuver = maneuver;
  request.own_vx_mps = 8.0;
  request.horizon_s = 8.0;
  request.sample_dt_s = 0.2;
  request.maneuver_duration_s = 4.0;
  request.turn_angle_deg = 45.0;
  request.min_turn_speed_mps = 1.0;
  request.climb_speed_mps = 3.0;
  request.gravity_mps2 = 9.80665;
  return autonomymind::SmoothTrajectoryGenerator{}.evaluate(request);
}

autonomymind::RtaRequest base() {
  autonomymind::RtaRequest request;
  request.trajectory = trajectory();
  request.sample_dt_s = 0.2;
  request.own_x_m = 0.0;
  request.own_y_m = 0.0;
  request.own_z_m = 100.0;
  request.own_vx_mps = 8.0;
  request.threat_rel_x_m = 100.0;
  request.threat_rel_y_m = 100.0;
  request.threat_rel_z_m = 0.0;

  request.constraints.min_separation_m = 15.0;
  request.constraints.terrain_elevation_m = 0.0;
  request.constraints.min_terrain_clearance_m = 20.0;
  request.constraints.geofence_min_x_m = -1000.0;
  request.constraints.geofence_max_x_m = 1000.0;
  request.constraints.geofence_min_y_m = -1000.0;
  request.constraints.geofence_max_y_m = 1000.0;
  request.constraints.geofence_min_z_m = 0.0;
  request.constraints.geofence_max_z_m = 1000.0;
  request.constraints.max_speed_mps = 30.0;
  request.constraints.max_vertical_speed_mps = 10.0;
  request.constraints.max_accel_mps2 = 10.0;
  request.constraints.max_jerk_mps3 = 30.0;
  request.constraints.max_yaw_rate_dps = 60.0;
  request.constraints.max_bank_angle_deg = 60.0;
  return request;
}

}  // namespace

int main() {
  using namespace autonomymind;

  const FullRtaChecker checker;

  // Nominal simulated candidate: every assurance domain passes.
  const auto nominal = checker.evaluate(base());
  if (!nominal.safe ||
      !nominal.traffic_safe ||
      !nominal.terrain_safe ||
      !nominal.geofence_safe ||
      !nominal.envelope_safe) {
    return 1;
  }

  // Simulated near traffic: only the traffic gate must reject the candidate.
  auto traffic = base();
  traffic.threat_rel_x_m = 5.0;
  traffic.threat_rel_y_m = 0.0;
  const auto traffic_result = checker.evaluate(traffic);
  if (traffic_result.safe || traffic_result.traffic_safe) {
    return 2;
  }

  // Exactly 15 m is the inclusive traffic boundary and must pass.
  auto traffic_boundary = base();
  traffic_boundary.trajectory = trajectory(SmoothManeuver::CLIMB);
  traffic_boundary.threat_rel_x_m = 0.0;
  traffic_boundary.threat_rel_y_m = 15.0;
  traffic_boundary.relative_vx_mps = 0.0;
  const auto traffic_boundary_result = checker.evaluate(traffic_boundary);
  if (!traffic_boundary_result.traffic_safe) {
    return 3;
  }

  // A point just inside the boundary must fail.
  traffic_boundary.threat_rel_y_m = 14.999;
  const auto traffic_below_result = checker.evaluate(traffic_boundary);
  if (traffic_below_result.traffic_safe) {
    return 4;
  }

  // Simulated terrain violation.
  auto terrain = base();
  terrain.constraints.terrain_elevation_m = 90.0;
  terrain.constraints.min_terrain_clearance_m = 20.0;
  const auto terrain_result = checker.evaluate(terrain);
  if (terrain_result.safe || terrain_result.terrain_safe) {
    return 5;
  }

  // Simulated 3D geofence violation.
  auto fence = base();
  fence.constraints.geofence_max_x_m = 10.0;
  const auto fence_result = checker.evaluate(fence);
  if (fence_result.safe || fence_result.geofence_safe) {
    return 6;
  }

  // Simulated flight-envelope violation.
  auto envelope = base();
  envelope.constraints.max_yaw_rate_dps = 1.0;
  const auto envelope_result = checker.evaluate(envelope);
  if (envelope_result.safe || envelope_result.envelope_safe) {
    return 7;
  }

  // Combined unsafe scenario proves fail-closed behavior: a candidate with
  // multiple simultaneous violations must never be marked safe.
  auto combined = base();
  combined.threat_rel_x_m = 5.0;
  combined.threat_rel_y_m = 0.0;
  combined.constraints.terrain_elevation_m = 95.0;
  combined.constraints.geofence_max_x_m = 4.0;
  combined.constraints.max_bank_angle_deg = 1.0;
  const auto combined_result = checker.evaluate(combined);
  if (combined_result.safe ||
      combined_result.traffic_safe ||
      combined_result.terrain_safe ||
      combined_result.geofence_safe ||
      combined_result.envelope_safe) {
    return 8;
  }

  std::cout
      << "PASS: simulated Full RTA nominal, boundaries, independent gates, "
         "and combined fail-closed scenario\n";
  return 0;
}
