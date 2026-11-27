#include "autonomymind/vehicle_rta.hpp"
#include <limits>
using namespace autonomymind;
int main() {
  VehicleProfile p;
  p.id="synthetic-test"; p.kind=VehicleKind::Rover; p.max_state_age_s=1;
  p.max_ground_height_error_m=0.1;
  auto& l=p.limits;
  l.min_separation_m=1; l.geofence_min_x_m=-100; l.geofence_max_x_m=100;
  l.geofence_min_y_m=-100; l.geofence_max_y_m=100;
  l.geofence_min_z_m=-1; l.geofence_max_z_m=100;
  l.max_speed_mps=10; l.max_vertical_speed_mps=5; l.max_accel_mps2=10;
  l.max_jerk_mps3=10; l.max_yaw_rate_dps=90; l.max_bank_angle_deg=45;
  RtaRequest r; r.sample_dt_s=1; r.threat_rel_x_m=50;
  r.trajectory.valid=true; r.trajectory.dynamically_feasible=true; r.trajectory.point_count=2;
  VehicleContext c{0,true,true,true}; ReferenceVehicleModel model; VehicleRta gate;
  if(!gate.evaluate({r},p,c,model).accepted) return 1;
  p.kind=VehicleKind::FixedWing; p.min_horizontal_speed_mps=2;
  if(gate.evaluate({r},p,c,model).accepted) return 2;
  p.kind=VehicleKind::Multirotor;
  if(!gate.evaluate({r},p,c,model).accepted) return 3;
  c.state_age_s=2;
  if(gate.evaluate({r},p,c,model).accepted) return 4;
  c.state_age_s=0; auto near=r; near.threat_rel_x_m=0;
  if(gate.evaluate({r,near},p,c,model).accepted) return 5;
  r.trajectory.point_count=FM_TRAJECTORY_POINTS+1;
  if(gate.evaluate({r},p,c,model).accepted) return 6;
  return 0;
}
