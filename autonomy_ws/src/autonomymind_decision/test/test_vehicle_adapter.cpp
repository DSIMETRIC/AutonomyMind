#include "autonomymind/vehicle_adapter.hpp"
using namespace autonomymind;
int main() {
  AdapterSnapshot s;
  s.vehicle_id="sim-rover"; s.frame_id="world_enu"; s.sequence=1;
  s.kind=VehicleKind::Rover; s.health={0,true,true,true};
  SimulationAdapter adapter(s);
  VehicleProfile p; p.id="synthetic"; p.kind=s.kind; p.max_state_age_s=1;
  p.max_ground_height_error_m=0.1;
  auto& l=p.limits;
  l.min_separation_m=1; l.geofence_min_x_m=-100; l.geofence_max_x_m=100;
  l.geofence_min_y_m=-100; l.geofence_max_y_m=100;
  l.geofence_min_z_m=-1; l.geofence_max_z_m=100;
  l.max_speed_mps=10; l.max_vertical_speed_mps=5; l.max_accel_mps2=10;
  l.max_jerk_mps3=10; l.max_yaw_rate_dps=90; l.max_bank_angle_deg=45;
  RtaRequest r; r.sample_dt_s=1; r.threat_rel_x_m=50;
  r.trajectory.valid=true; r.trajectory.dynamically_feasible=true; r.trajectory.point_count=2;
  ReferenceVehicleModel model;
  if(adapter.assess_and_record("wrong",1,{r},p,model).recorded) return 1;
  if(!adapter.assess_and_record(s.vehicle_id,1,{r},p,model).recorded) return 2;
  if(!adapter.recorded_candidate()) return 3;
  if(adapter.assess_and_record(s.vehicle_id,1,{r},p,model).recorded) return 4;
  s.sequence=2; adapter.update(s); r.own_x_m=1;
  if(adapter.assess_and_record(s.vehicle_id,2,{r},p,model).recorded) return 5;
  r.own_x_m=0; r.threat_rel_x_m=0;
  if(adapter.assess_and_record(s.vehicle_id,2,{r},p,model).recorded) return 6;
  s.sequence=3; s.health.communication_valid=false; adapter.update(s); r.threat_rel_x_m=50;
  if(adapter.assess_and_record(s.vehicle_id,3,{r},p,model).recorded) return 7;
  return 0;
}
