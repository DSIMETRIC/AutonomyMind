#include "autonomymind/trajectory_generator.hpp"
#include <cmath>
#include <iostream>

int main(){
  using namespace autonomymind;
  SmoothTrajectoryRequest q;
  q.own_vx_mps=10.0;q.own_vy_mps=0.0;q.own_vz_mps=0.0;
  q.horizon_s=8.0;q.sample_dt_s=0.1;q.maneuver_duration_s=4.0;
  q.turn_angle_deg=45.0;q.min_turn_speed_mps=1.0;q.climb_speed_mps=3.0;q.gravity_mps2=9.80665;
  const SmoothTrajectoryGenerator g;
  q.maneuver=SmoothManeuver::LEFT;const auto left=g.evaluate(q);
  q.maneuver=SmoothManeuver::RIGHT;const auto right=g.evaluate(q);
  if(!left.valid||!right.valid)return 1;
  if(left.point_count<20||right.point_count!=left.point_count)return 2;
  if(!(left.trajectory_y_m[left.point_count-1]>1.0))return 3;
  if(!(right.trajectory_y_m[right.point_count-1]<-1.0))return 4;
  if(std::abs(left.trajectory_x_m[left.point_count-1]-right.trajectory_x_m[right.point_count-1])>0.5)return 5;
  if(left.peak_yaw_rate_dps<=0.0||left.peak_bank_angle_deg<=0.0)return 6;
  if(std::abs(left.peak_yaw_rate_dps-right.peak_yaw_rate_dps)>1e-9)return 7;
  q.maneuver=SmoothManeuver::CLIMB;const auto climb=g.evaluate(q);
  if(!climb.valid||climb.trajectory_z_m[climb.point_count-1]<=1.0)return 8;
  if(climb.peak_vertical_speed_mps<2.9)return 9;
  auto hover=q;hover.maneuver=SmoothManeuver::LEFT;hover.own_vx_mps=0.1;hover.min_turn_speed_mps=1.0;
  if(g.evaluate(hover).valid)return 10;
  std::cout<<"PASS: coordinated-turn LEFT/RIGHT and smooth CLIMB trajectories\n";
  return 0;
}
