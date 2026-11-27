#include "autonomymind/behavior_tree.hpp"
#include <iostream>

namespace {
autonomymind::CollisionAvoidanceRequest base_request() {
  autonomymind::CollisionAvoidanceRequest r;
  r.traffic_conflict=true;
  r.rel_x_m=40.0;
  r.rel_y_m=10.0;
  r.rel_z_m=0.0;
  r.relative_vx_mps=-10.0;
  r.relative_vy_mps=0.0;
  r.relative_vz_mps=0.0;
  r.min_separation_m=15.0;
  r.prediction_horizon_s=8.0;
  r.prediction_dt_s=0.2;
  r.climb_speed_mps=3.0;
  r.turn_angle_deg=45.0;
  r.min_turn_speed_mps=1.0;
  r.gravity_mps2=9.80665;
  r.own_vx_mps=5.0;
  r.own_vy_mps=0.0;
  r.own_vz_mps=0.0;
  r.maneuver_duration_s=4.0;
  r.own_z_m=100.0;
  r.terrain_elevation_m=0.0;
  r.min_terrain_clearance_m=20.0;
  r.geofence_min_x_m=-1000.0; r.geofence_max_x_m=1000.0;
  r.geofence_min_y_m=-1000.0; r.geofence_max_y_m=1000.0;
  r.geofence_min_z_m=0.0; r.geofence_max_z_m=1000.0;
  r.max_speed_mps=50.0; r.max_vertical_speed_mps=20.0;
  r.max_accel_mps2=20.0; r.max_jerk_mps3=50.0;
  r.max_yaw_rate_dps=90.0; r.max_bank_angle_deg=80.0;
  return r;
}
}

int main(){
  using namespace autonomymind;
  const CollisionAvoidanceTree tree;

  auto no_conflict=base_request();
  no_conflict.traffic_conflict=false;
  if(tree.tick(no_conflict).status!=BehaviorStatus::FAILURE) return 1;

  const auto a=tree.tick(base_request());
  if(a.status!=BehaviorStatus::SUCCESS) return 2;
  if(a.safe_candidate_count<=0) return 3;
  if(a.selected_predicted_min_separation_m<15.0) return 4;
  if(a.selected_index<0 || a.selected_index>=3) return 8;
  const auto& selected=a.candidates[static_cast<std::size_t>(a.selected_index)];
  if(!selected.dynamically_feasible ||
     !selected.traffic_safe ||
     !selected.terrain_safe ||
     !selected.geofence_safe ||
     !selected.envelope_safe ||
     !selected.rta_safe) return 9;
  if(selected.point_count<=1 || selected.point_count>FM_TRAJECTORY_POINTS) return 10;
  if(selected.peak_speed_mps<=0.0 || selected.peak_accel_mps2<=0.0 ||
     selected.peak_jerk_mps3<=0.0) return 11;

  // Simulated terrain constraint: all candidates violate clearance, therefore
  // no maneuver is allowed to reach operational arbitration.
  auto terrain_blocked=base_request();
  terrain_blocked.terrain_elevation_m=95.0;
  terrain_blocked.min_terrain_clearance_m=20.0;
  const auto terrain_result=tree.tick(terrain_blocked);
  if(terrain_result.status!=BehaviorStatus::FAILURE ||
     terrain_result.safe_candidate_count!=0) return 12;

  // Simulated geofence constraint: a narrow x-volume blocks every trajectory.
  auto fence_blocked=base_request();
  fence_blocked.geofence_max_x_m=2.0;
  const auto fence_result=tree.tick(fence_blocked);
  if(fence_result.status!=BehaviorStatus::FAILURE ||
     fence_result.safe_candidate_count!=0) return 13;

  // Simulated flight-envelope constraint: an unrealistically small speed limit
  // blocks all candidates and proves the envelope gate participates in selection.
  auto envelope_blocked=base_request();
  envelope_blocked.max_speed_mps=1.0;
  const auto envelope_result=tree.tick(envelope_blocked);
  if(envelope_result.status!=BehaviorStatus::FAILURE ||
     envelope_result.safe_candidate_count!=0) return 14;

  // Force every candidate below a deliberately impossible RTA separation.
  auto blocked=base_request();
  blocked.min_separation_m=1000.0;
  const auto b=tree.tick(blocked);
  if(b.status!=BehaviorStatus::FAILURE) return 5;
  if(b.safe_candidate_count!=0) return 6;

  // Same input must always produce exactly the same selected maneuver/evidence.
  const auto c=tree.tick(base_request());
  if(a.candidate_decision!=c.candidate_decision ||
     a.candidate_guidance!=c.candidate_guidance ||
     a.selected_predicted_min_separation_m!=c.selected_predicted_min_separation_m)
    return 7;

  std::cout<<"PASS: simulated candidate generation, all Full RTA gates, fail-closed behavior and deterministic selection\n";
  return 0;
}
