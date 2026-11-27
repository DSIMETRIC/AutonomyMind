#include "autonomymind/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>

namespace autonomymind {
namespace {

constexpr double kRadToDeg = 57.2957795130823208768;
constexpr double kDegToRad = 0.01745329251994329577;
constexpr double kEpsilon = 1e-9;

// Quintic smoothstep provides zero first and second derivative at both ends.
// That avoids instantaneous heading-rate/acceleration steps in the commanded
// turn profile.
double smoothstep5(double u) {
  const double u2 = u * u;
  const double u3 = u2 * u;
  const double u4 = u3 * u;
  const double u5 = u4 * u;
  return 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
}

double smoothstep5FirstDerivative(double u) {
  return 30.0 * u * u -
         60.0 * u * u * u +
         30.0 * u * u * u * u;
}

double smoothstep5SecondDerivative(double u) {
  return 60.0 * u -
         180.0 * u * u +
         120.0 * u * u * u;
}

}  // namespace

SmoothTrajectoryResult SmoothTrajectoryGenerator::evaluate(
    const SmoothTrajectoryRequest& request) const {
  SmoothTrajectoryResult output;

  // Invalid shaping parameters fail closed. RTA must not receive a trajectory
  // marked valid when the time horizon or physical constants are unusable.
  const bool invalid_request =
      request.horizon_s <= 0.0 ||
      request.sample_dt_s <= 0.0 ||
      request.sample_dt_s > request.horizon_s ||
      request.maneuver_duration_s <= 0.0 ||
      request.maneuver_duration_s > request.horizon_s ||
      request.climb_speed_mps <= 0.0 ||
      request.gravity_mps2 <= 0.0 ||
      request.turn_angle_deg <= 0.0 ||
      request.min_turn_speed_mps <= 0.0;

  if (invalid_request) {
    return output;
  }

  const double horizontal_speed_mps =
      std::hypot(request.own_vx_mps, request.own_vy_mps);

  // The coordinated-turn approximation is not valid below the configured
  // minimum horizontal speed, so LEFT/RIGHT candidates are rejected.
  const bool is_turn =
      request.maneuver == SmoothManeuver::LEFT ||
      request.maneuver == SmoothManeuver::RIGHT;

  if (is_turn && horizontal_speed_mps < request.min_turn_speed_mps) {
    return output;
  }

  const double initial_heading_rad =
      std::atan2(request.own_vy_mps, request.own_vx_mps);

  const double turn_sign =
      request.maneuver == SmoothManeuver::RIGHT ? -1.0 : 1.0;

  const double turn_angle_rad =
      request.turn_angle_deg * kDegToRad;

  const double maneuver_duration_s =
      request.maneuver_duration_s;

  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;

  double previous_vx_mps = request.own_vx_mps;
  double previous_vy_mps = request.own_vy_mps;
  double previous_vz_mps = request.own_vz_mps;

  double previous_ax_mps2 = 0.0;
  double previous_ay_mps2 = 0.0;
  double previous_az_mps2 = 0.0;

  bool have_previous_sample = false;
  int point_count = 0;

  for (double time_s = 0.0;
       time_s <= request.horizon_s + kEpsilon;
       time_s += request.sample_dt_s) {
    const double maneuver_time_s =
        std::min(time_s, maneuver_duration_s);
    const double u =
        maneuver_time_s / maneuver_duration_s;

    const double q = smoothstep5(u);
    const double q_first = smoothstep5FirstDerivative(u);
    const double q_second = smoothstep5SecondDerivative(u);

    double vx_mps = request.own_vx_mps;
    double vy_mps = request.own_vy_mps;
    double vz_mps = request.own_vz_mps;

    if (is_turn) {
      // Coordinated-turn kinematic approximation:
      // preserve horizontal speed while smoothly rotating the velocity vector.
      // This is deliberately not a six-degree-of-freedom aircraft model.
      const double heading_rad =
          initial_heading_rad + turn_sign * turn_angle_rad * q;

      vx_mps = horizontal_speed_mps * std::cos(heading_rad);
      vy_mps = horizontal_speed_mps * std::sin(heading_rad);

      const double yaw_rate_rad_s =
          maneuver_time_s < maneuver_duration_s + kEpsilon
              ? turn_sign * turn_angle_rad * q_first /
                    maneuver_duration_s
              : 0.0;

      // For a coordinated turn, lateral acceleration is v * yaw_rate.
      // Bank angle is the kinematic estimate atan(a_lateral / g).
      const double lateral_accel_mps2 =
          horizontal_speed_mps * std::abs(yaw_rate_rad_s);
      const double bank_angle_deg =
          std::atan2(lateral_accel_mps2, request.gravity_mps2) *
          kRadToDeg;

      output.peak_yaw_rate_dps = std::max(
          output.peak_yaw_rate_dps,
          std::abs(yaw_rate_rad_s) * kRadToDeg);

      output.peak_bank_angle_deg = std::max(
          output.peak_bank_angle_deg,
          std::abs(bank_angle_deg));
    } else {
      // CLIMB keeps the horizontal velocity and smoothly adds the requested
      // vertical speed over the maneuver duration.
      const double additional_vz_mps =
          request.climb_speed_mps * q;
      vz_mps = request.own_vz_mps + additional_vz_mps;
    }

    if (have_previous_sample) {
      const double dt_s = request.sample_dt_s;

      // Deterministic trapezoidal integration converts velocity samples into
      // candidate positions relative to the ownship initial position.
      x_m += 0.5 * (previous_vx_mps + vx_mps) * dt_s;
      y_m += 0.5 * (previous_vy_mps + vy_mps) * dt_s;
      z_m += 0.5 * (previous_vz_mps + vz_mps) * dt_s;

      const double ax_mps2 =
          (vx_mps - previous_vx_mps) / dt_s;
      const double ay_mps2 =
          (vy_mps - previous_vy_mps) / dt_s;
      const double az_mps2 =
          (vz_mps - previous_vz_mps) / dt_s;

      const double acceleration_mps2 = std::sqrt(
          ax_mps2 * ax_mps2 +
          ay_mps2 * ay_mps2 +
          az_mps2 * az_mps2);

      output.peak_accel_mps2 =
          std::max(output.peak_accel_mps2, acceleration_mps2);

      if (point_count > 1) {
        const double jerk_x_mps3 =
            (ax_mps2 - previous_ax_mps2) / dt_s;
        const double jerk_y_mps3 =
            (ay_mps2 - previous_ay_mps2) / dt_s;
        const double jerk_z_mps3 =
            (az_mps2 - previous_az_mps2) / dt_s;

        const double jerk_mps3 = std::sqrt(
            jerk_x_mps3 * jerk_x_mps3 +
            jerk_y_mps3 * jerk_y_mps3 +
            jerk_z_mps3 * jerk_z_mps3);

        output.peak_jerk_mps3 =
            std::max(output.peak_jerk_mps3, jerk_mps3);
      }

      previous_ax_mps2 = ax_mps2;
      previous_ay_mps2 = ay_mps2;
      previous_az_mps2 = az_mps2;
    }

    if (point_count < FM_TRAJECTORY_POINTS) {
      output.trajectory_x_m[point_count] = x_m;
      output.trajectory_y_m[point_count] = y_m;
      output.trajectory_z_m[point_count] = z_m;
      ++point_count;
    }

    output.peak_speed_mps = std::max(
        output.peak_speed_mps,
        std::sqrt(
            vx_mps * vx_mps +
            vy_mps * vy_mps +
            vz_mps * vz_mps));

    output.peak_vertical_speed_mps = std::max(
        output.peak_vertical_speed_mps,
        std::abs(vz_mps));

    previous_vx_mps = vx_mps;
    previous_vy_mps = vy_mps;
    previous_vz_mps = vz_mps;
    have_previous_sample = true;

    if (is_turn && maneuver_time_s < maneuver_duration_s + kEpsilon) {
      // q'' controls heading acceleration. v * |heading acceleration| is used
      // as the turn-profile contribution to peak jerk.
      const double yaw_accel_rad_s2 =
          std::abs(
              turn_angle_rad * q_second /
              (maneuver_duration_s * maneuver_duration_s));

      output.peak_jerk_mps3 = std::max(
          output.peak_jerk_mps3,
          horizontal_speed_mps * yaw_accel_rad_s2);
    }
  }

  output.point_count = point_count;
  output.valid = point_count > 1;
  output.dynamically_feasible = output.valid;
  return output;
}

}  // namespace autonomymind
