#include "autonomymind/vehicle_rta.hpp"
#include <cmath>

namespace autonomymind {
namespace {
bool finite(double v) { return std::isfinite(v); }
bool positive(double v) { return finite(v) && v > 0; }
bool nonnegative(double v) { return finite(v) && v >= 0; }
bool valid_limits(const RtaConstraints& c) {
  return positive(c.min_separation_m) && finite(c.terrain_elevation_m) &&
    nonnegative(c.min_terrain_clearance_m) &&
    finite(c.geofence_min_x_m) && finite(c.geofence_max_x_m) &&
    finite(c.geofence_min_y_m) && finite(c.geofence_max_y_m) &&
    finite(c.geofence_min_z_m) && finite(c.geofence_max_z_m) &&
    c.geofence_min_x_m < c.geofence_max_x_m &&
    c.geofence_min_y_m < c.geofence_max_y_m &&
    c.geofence_min_z_m < c.geofence_max_z_m &&
    positive(c.max_speed_mps) && positive(c.max_vertical_speed_mps) &&
    positive(c.max_accel_mps2) && positive(c.max_jerk_mps3) &&
    positive(c.max_yaw_rate_dps) && positive(c.max_bank_angle_deg);
}
bool valid_request(const RtaRequest& r) {
  const auto& t=r.trajectory;
  if (!t.valid || !t.dynamically_feasible || t.point_count<2 ||
      t.point_count>FM_TRAJECTORY_POINTS || !positive(r.sample_dt_s)) return false;
  for (double v : {r.own_x_m,r.own_y_m,r.own_z_m,r.own_vx_mps,r.own_vy_mps,r.own_vz_mps,
      r.threat_rel_x_m,r.threat_rel_y_m,r.threat_rel_z_m,r.relative_vx_mps,r.relative_vy_mps,r.relative_vz_mps})
    if (!finite(v)) return false;
  for (double v : {t.peak_speed_mps,t.peak_vertical_speed_mps,t.peak_accel_mps2,t.peak_jerk_mps3,t.peak_yaw_rate_dps,t.peak_bank_angle_deg})
    if (!nonnegative(v)) return false;
  for (int i=0;i<t.point_count;++i)
    if (!finite(t.trajectory_x_m[i]) || !finite(t.trajectory_y_m[i]) || !finite(t.trajectory_z_m[i])) return false;
  return true;
}
bool same_candidate(const RtaRequest& a,const RtaRequest& b) {
  if (a.sample_dt_s!=b.sample_dt_s || a.own_x_m!=b.own_x_m || a.own_y_m!=b.own_y_m || a.own_z_m!=b.own_z_m ||
      a.own_vx_mps!=b.own_vx_mps || a.own_vy_mps!=b.own_vy_mps || a.own_vz_mps!=b.own_vz_mps) return false;
  const auto& x=a.trajectory; const auto& y=b.trajectory;
  if(x.point_count!=y.point_count || x.peak_speed_mps!=y.peak_speed_mps ||
     x.peak_vertical_speed_mps!=y.peak_vertical_speed_mps || x.peak_accel_mps2!=y.peak_accel_mps2 ||
     x.peak_jerk_mps3!=y.peak_jerk_mps3 || x.peak_yaw_rate_dps!=y.peak_yaw_rate_dps ||
     x.peak_bank_angle_deg!=y.peak_bank_angle_deg) return false;
  for(int i=0;i<x.point_count;++i)
    if(x.trajectory_x_m[i]!=y.trajectory_x_m[i] || x.trajectory_y_m[i]!=y.trajectory_y_m[i] ||
       x.trajectory_z_m[i]!=y.trajectory_z_m[i]) return false;
  return true;
}
}
bool ReferenceVehicleModel::supports(VehicleKind k) const {
  return k==VehicleKind::FixedWing || k==VehicleKind::Multirotor || k==VehicleKind::Rover;
}
bool ReferenceVehicleModel::envelope(const RtaRequest& r,const VehicleProfile& p) const {
  const auto& t=r.trajectory;
  for(int i=0;i<t.point_count;++i) {
    if(p.kind==VehicleKind::Rover &&
       std::abs(r.own_z_m+t.trajectory_z_m[i]-p.limits.terrain_elevation_m)>p.max_ground_height_error_m) return false;
    if(i==0) continue;
    const double vx=(t.trajectory_x_m[i]-t.trajectory_x_m[i-1])/r.sample_dt_s;
    const double vy=(t.trajectory_y_m[i]-t.trajectory_y_m[i-1])/r.sample_dt_s;
    const double vz=(t.trajectory_z_m[i]-t.trajectory_z_m[i-1])/r.sample_dt_s;
    if(std::hypot(std::hypot(vx,vy),vz)>p.limits.max_speed_mps || std::abs(vz)>p.limits.max_vertical_speed_mps) return false;
    if(p.kind==VehicleKind::FixedWing && std::hypot(vx,vy)<p.min_horizontal_speed_mps) return false;
  }
  return true;
}
VehicleAssessment VehicleRta::evaluate(const std::vector<RtaRequest>& requests,
    const VehicleProfile& p,const VehicleContext& c,const VehicleModel& model) const {
  VehicleAssessment out;
  if(p.id.empty() || !valid_limits(p.limits) || !positive(p.max_state_age_s) ||
     !nonnegative(p.min_horizontal_speed_mps) || !nonnegative(p.max_ground_height_error_m) ||
     (p.kind==VehicleKind::FixedWing && !positive(p.min_horizontal_speed_mps)) || !model.supports(p.kind)) {
    out.reason="invalid_or_unsupported_profile"; return out;
  }
  if(!c.state_valid || !c.environment_valid || !c.communication_valid ||
     !nonnegative(c.state_age_s) || c.state_age_s>p.max_state_age_s || requests.empty()) {
    out.reason="missing_or_stale_context"; return out;
  }
  for(auto r:requests) {
    if(!valid_request(r) || !same_candidate(requests.front(),r)) {out.reason="invalid_candidate"; return out;}
    r.constraints=p.limits;
    const auto result=FullRtaChecker{}.evaluate(r);
    out.common_checks.push_back(result);
    if(!result.safe || !model.envelope(r,p)) {out.reason="constraint_rejected"; return out;}
  }
  out.accepted=true; out.reason="reference_checks_passed"; return out;
}
}
