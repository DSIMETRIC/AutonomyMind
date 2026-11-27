#include "autonomymind/mission_behavior_tree.hpp"
#include <iostream>

using namespace autonomymind;

namespace {
MissionDecisionRequest nominal(){
  MissionDecisionRequest r;
  r.nav_sources_valid=2;
  r.nav_minimum_operational_sources=1;
  r.nav_sources_required=2;
  r.nav_integrity=1.0;
  r.nav_integrity_hold_min=0.2;
  r.nav_integrity_degrade_min=0.5;
  r.rejoin_enter_m=3.0;
  r.rejoin_heading_hold_deg=5.0;
  r.collision.traffic_conflict=true;
  r.collision.rel_x_m=40.0;
  r.collision.rel_y_m=10.0;
  r.collision.relative_vx_mps=-10.0;
  r.collision.min_separation_m=15.0;
  r.collision.prediction_horizon_s=8.0;
  r.collision.prediction_dt_s=0.2;
  r.collision.climb_speed_mps=3.0;
  r.collision.turn_angle_deg=45.0;
  r.collision.min_turn_speed_mps=1.0;
  r.collision.gravity_mps2=9.80665;
  r.collision.own_vx_mps=5.0;
  r.collision.own_vy_mps=0.0;
  r.collision.own_vz_mps=0.0;
  r.collision.maneuver_duration_s=4.0;
  r.collision.own_z_m=100.0;
  r.collision.terrain_elevation_m=0.0;
  r.collision.min_terrain_clearance_m=20.0;
  r.collision.geofence_min_x_m=-1000.0; r.collision.geofence_max_x_m=1000.0;
  r.collision.geofence_min_y_m=-1000.0; r.collision.geofence_max_y_m=1000.0;
  r.collision.geofence_min_z_m=0.0; r.collision.geofence_max_z_m=1000.0;
  r.collision.max_speed_mps=50.0; r.collision.max_vertical_speed_mps=20.0;
  r.collision.max_accel_mps2=20.0; r.collision.max_jerk_mps3=50.0;
  r.collision.max_yaw_rate_dps=90.0; r.collision.max_bank_angle_deg=80.0;
  return r;
}
bool is_avoid(Decision d){
  return d==Decision::AVOID_LEFT || d==Decision::AVOID_RIGHT ||
         d==Decision::AVOID_CLIMB;
}
}

int main(){
  const MissionDecisionTree tree;

  { auto r=nominal(); auto o=tree.tick(r);
    if(o.decision!=Decision::CONTINUE_MISSION) return 1; }

  { auto r=nominal(); r.nav_sources_valid=0; auto o=tree.tick(r);
    if(o.decision!=Decision::HOLD || o.active_rule!=ActiveRule::NAV_UNAVAILABLE) return 2; }

  { auto r=nominal(); r.nav_integrity=0.1; auto o=tree.tick(r);
    if(o.decision!=Decision::HOLD || o.active_rule!=ActiveRule::NAV_INTEGRITY_HOLD) return 3; }

  { auto r=nominal(); r.nav_disagreement_active=true; auto o=tree.tick(r);
    if(o.decision!=Decision::VERIFY_NAV) return 4; }

  { auto r=nominal(); r.nav_sources_valid=1; auto o=tree.tick(r);
    if(o.decision!=Decision::DEGRADE_NAV ||
       o.active_rule!=ActiveRule::NAV_REDUNDANCY_DEGRADED) return 5; }

  { auto r=nominal(); r.nav_integrity=0.4; auto o=tree.tick(r);
    if(o.decision!=Decision::DEGRADE_NAV ||
       o.active_rule!=ActiveRule::NAV_INTEGRITY_DEGRADED) return 6; }

  { auto r=nominal(); r.mission_available=true; r.cross_track_error_m=4.0;
    r.rejoin_heading_error_deg=20.0; auto o=tree.tick(r);
    if(o.decision!=Decision::REJOIN_PATH || o.guidance!=Guidance::TURN_LEFT) return 7; }

  { auto r=nominal(); r.traffic_conflict=true; auto o=tree.tick(r);
    if(!is_avoid(o.decision)) return 8; }

  { auto r=nominal(); r.traffic_conflict=true;
    r.collision.min_separation_m=1000.0; auto o=tree.tick(r);
    if(o.decision!=Decision::HOLD || o.active_rule!=ActiveRule::TRAFFIC_CONFLICT) return 9; }

  // Priority: critical NAV must win even when traffic, disagreement and rejoin are active.
  { auto r=nominal(); r.nav_sources_valid=0; r.traffic_conflict=true;
    r.nav_disagreement_active=true; r.mission_available=true; r.cross_track_error_m=50.0;
    auto o=tree.tick(r);
    if(o.decision!=Decision::HOLD || o.active_rule!=ActiveRule::NAV_UNAVAILABLE) return 10; }

  // Priority: traffic must win over disagreement, degraded NAV and rejoin.
  { auto r=nominal(); r.traffic_conflict=true; r.nav_disagreement_active=true;
    r.mission_available=true; r.cross_track_error_m=50.0; auto o=tree.tick(r);
    if(!is_avoid(o.decision) || o.active_rule!=ActiveRule::TRAFFIC_CONFLICT) return 11; }

  // Recovery: once all abnormal conditions clear, mission returns nominal.
  { auto r=nominal(); r.traffic_conflict=false; r.nav_disagreement_active=false;
    r.mission_available=true; r.cross_track_error_m=0.5; auto o=tree.tick(r);
    if(o.decision!=Decision::CONTINUE_MISSION) return 12; }

  std::cout<<"PASS: all demo decisions, fallback behavior, priority and recovery\n";
  return 0;
}
