#include "autonomymind/rta.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autonomymind {
namespace {

double norm3(double x, double y, double z) {
  return std::sqrt(x * x + y * y + z * z);
}

bool within(double value, double lower, double upper) {
  return value >= lower && value <= upper;
}

}  // namespace

RtaResult FullRtaChecker::evaluate(const RtaRequest& request) const {
  RtaResult output;
  const auto& trajectory = request.trajectory;
  const auto& constraints = request.constraints;

  // Reject incomplete or physically meaningless RTA requests. A malformed
  // request must never be interpreted as a safe trajectory.
  const bool invalid_request =
      !trajectory.valid ||
      trajectory.point_count <= 1 ||
      request.sample_dt_s <= 0.0 ||
      constraints.min_separation_m <= 0.0 ||
      constraints.min_terrain_clearance_m < 0.0 ||
      !(constraints.geofence_min_x_m < constraints.geofence_max_x_m) ||
      !(constraints.geofence_min_y_m < constraints.geofence_max_y_m) ||
      !(constraints.geofence_min_z_m < constraints.geofence_max_z_m) ||
      constraints.max_speed_mps <= 0.0 ||
      constraints.max_vertical_speed_mps <= 0.0 ||
      constraints.max_accel_mps2 <= 0.0 ||
      constraints.max_jerk_mps3 <= 0.0 ||
      constraints.max_yaw_rate_dps <= 0.0 ||
      constraints.max_bank_angle_deg <= 0.0;

  if (invalid_request) {
    return output;
  }

  output.predicted_min_separation_m =
      std::numeric_limits<double>::infinity();
  output.minimum_terrain_clearance_m =
      std::numeric_limits<double>::infinity();
  output.terrain_safe = true;
  output.geofence_safe = true;

  // Relative velocity is converted back to threat inertial velocity so both
  // aircraft are propagated in the same reference frame.
  const double threat_vx_mps =
      request.relative_vx_mps + request.own_vx_mps;
  const double threat_vy_mps =
      request.relative_vy_mps + request.own_vy_mps;
  const double threat_vz_mps =
      request.relative_vz_mps + request.own_vz_mps;

  for (int index = 0; index < trajectory.point_count; ++index) {
    const double time_s =
        static_cast<double>(index) * request.sample_dt_s;

    // Candidate trajectory coordinates are relative to the ownship position at
    // the beginning of the RTA horizon.
    const double own_rel_x_m = trajectory.trajectory_x_m[index];
    const double own_rel_y_m = trajectory.trajectory_y_m[index];
    const double own_rel_z_m = trajectory.trajectory_z_m[index];

    // Propagate the traffic threat with constant velocity over the same
    // deterministic prediction horizon used by the candidate trajectory.
    const double threat_x_m =
        request.threat_rel_x_m + threat_vx_mps * time_s;
    const double threat_y_m =
        request.threat_rel_y_m + threat_vy_mps * time_s;
    const double threat_z_m =
        request.threat_rel_z_m + threat_vz_mps * time_s;

    const double separation_m = norm3(
        threat_x_m - own_rel_x_m,
        threat_y_m - own_rel_y_m,
        threat_z_m - own_rel_z_m);

    output.predicted_min_separation_m =
        std::min(output.predicted_min_separation_m, separation_m);

    // Convert the relative candidate point to world coordinates before
    // evaluating terrain clearance and the configured 3D geofence.
    const double world_x_m = request.own_x_m + own_rel_x_m;
    const double world_y_m = request.own_y_m + own_rel_y_m;
    const double world_z_m = request.own_z_m + own_rel_z_m;

    // Baseline terrain model: a flat elevation plane. This is intentionally
    // simple and must not be confused with a DEM/terrain-database implementation.
    const double terrain_clearance_m =
        world_z_m - constraints.terrain_elevation_m;

    output.minimum_terrain_clearance_m = std::min(
        output.minimum_terrain_clearance_m,
        terrain_clearance_m);

    if (terrain_clearance_m < constraints.min_terrain_clearance_m) {
      output.terrain_safe = false;
    }

    // Baseline geofence model: an axis-aligned 3D volume. Every sampled point
    // must remain inside all three configured bounds.
    const bool inside_geofence =
        within(
            world_x_m,
            constraints.geofence_min_x_m,
            constraints.geofence_max_x_m) &&
        within(
            world_y_m,
            constraints.geofence_min_y_m,
            constraints.geofence_max_y_m) &&
        within(
            world_z_m,
            constraints.geofence_min_z_m,
            constraints.geofence_max_z_m);

    if (!inside_geofence) {
      output.geofence_safe = false;
    }
  }

  // Traffic separation is inclusive at the configured minimum: exactly the
  // minimum separation is accepted; anything below it is rejected.
  output.traffic_safe =
      output.predicted_min_separation_m >= constraints.min_separation_m;

  // Flight-envelope assurance checks the extrema measured by the trajectory
  // generator. All limits must pass; RTA does not select a partially safe path.
  output.envelope_safe =
      trajectory.peak_speed_mps <= constraints.max_speed_mps &&
      trajectory.peak_vertical_speed_mps <=
          constraints.max_vertical_speed_mps &&
      trajectory.peak_accel_mps2 <= constraints.max_accel_mps2 &&
      trajectory.peak_jerk_mps3 <= constraints.max_jerk_mps3 &&
      trajectory.peak_yaw_rate_dps <= constraints.max_yaw_rate_dps &&
      trajectory.peak_bank_angle_deg <= constraints.max_bank_angle_deg;

  output.valid =
      std::isfinite(output.predicted_min_separation_m) &&
      std::isfinite(output.minimum_terrain_clearance_m);

  // Full RTA is fail-closed: a candidate is safe only when the request is
  // valid and traffic, terrain, geofence, and flight-envelope gates all pass.
  output.safe =
      output.valid &&
      output.traffic_safe &&
      output.terrain_safe &&
      output.geofence_safe &&
      output.envelope_safe;

  return output;
}

}  // namespace autonomymind
