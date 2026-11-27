#include "autonomymind/vehicle_adapter.hpp"
#include <autonomymind_interfaces/srv/assess_simulation_trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace autonomymind {
using Service=autonomymind_interfaces::srv::AssessSimulationTrajectory;
using Clock=std::chrono::steady_clock;
class SimulationRtaNode final:public rclcpp::Node {
 public:
  SimulationRtaNode():Node("simulation_rta") {
    id_=declare_parameter<std::string>("vehicle_id","sim-rover");
    const auto kind=declare_parameter<std::string>("vehicle_kind","rover");
    profile_.id="synthetic-"+kind;
    if(kind=="rover") profile_.kind=VehicleKind::Rover;
    else if(kind=="multirotor") profile_.kind=VehicleKind::Multirotor;
    else if(kind=="fixed_wing") profile_.kind=VehicleKind::FixedWing;
    else throw std::invalid_argument("unsupported reference vehicle kind");
    profile_.max_state_age_s=0.5;
    profile_.max_ground_height_error_m=0.1;
    profile_.min_horizontal_speed_mps=kind=="fixed_wing"?2.0:0.0;
    auto& l=profile_.limits;
    l.min_separation_m=1; l.geofence_min_x_m=-100; l.geofence_max_x_m=100;
    l.geofence_min_y_m=-100; l.geofence_max_y_m=100;
    l.geofence_min_z_m=-1; l.geofence_max_z_m=100;
    l.max_speed_mps=10; l.max_vertical_speed_mps=5; l.max_accel_mps2=10;
    l.max_jerk_mps3=10; l.max_yaw_rate_dps=90; l.max_bank_angle_deg=45;
    l.min_terrain_clearance_m=kind=="rover"?0.0:1.0;
    sub_=create_subscription<nav_msgs::msg::Odometry>("simulation/odometry",rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m){ odom_=m; received_=Clock::now(); });
    service_=create_service<Service>("simulation/assess_trajectory",
        [this](const std::shared_ptr<Service::Request> q,std::shared_ptr<Service::Response> a){assess(*q,*a);});
    RCLCPP_WARN(get_logger(),"Reference simulation assessment only; no actuator output");
  }
 private:
  void assess(const Service::Request& q,Service::Response& a) {
    a.sequence=q.sequence; a.profile_id=profile_.id; a.reason="invalid_or_missing_input";
    a.state_receipt_age_s=-1;
    if(!odom_) return;
    const auto age=std::chrono::duration<double>(Clock::now()-received_).count();
    a.state_receipt_age_s=age;
    const auto& m=*odom_;
    if(q.vehicle_id!=id_ || q.frame_id!="world_enu" || m.header.frame_id!=q.frame_id ||
       m.child_frame_id.empty() || q.odom_stamp_sec!=m.header.stamp.sec ||
       q.odom_stamp_nanosec!=m.header.stamp.nanosec || q.sequence<=last_accepted_ ||
       age>profile_.max_state_age_s || !q.environment_valid || !q.communication_valid) return;
    const auto n=q.x_m.size();
    if(n<2 || n>FM_TRAJECTORY_POINTS || q.y_m.size()!=n || q.z_m.size()!=n ||
       q.threat_position_m.empty() || q.threat_position_m.size()%3!=0 ||
       q.threat_position_m.size()!=q.threat_velocity_mps.size() ||
       q.threat_position_m.size()>300) return;
    // Odometry twist is expressed in child_frame_id. Rotate into world ENU.
    const auto& o=m.pose.pose.orientation;
    const double norm=o.x*o.x+o.y*o.y+o.z*o.z+o.w*o.w;
    if(!std::isfinite(norm) || std::abs(norm-1)>1e-3) return;
    const auto& v=m.twist.twist.linear;
    const double tx=2*(o.y*v.z-o.z*v.y),ty=2*(o.z*v.x-o.x*v.z),tz=2*(o.x*v.y-o.y*v.x);
    AdapterSnapshot s;
    s.vehicle_id=id_; s.frame_id=q.frame_id; s.sequence=q.sequence; s.kind=profile_.kind;
    s.health={age,true,q.environment_valid,q.communication_valid};
    s.x=m.pose.pose.position.x; s.y=m.pose.pose.position.y; s.z=m.pose.pose.position.z;
    s.vx=v.x+o.w*tx+o.y*tz-o.z*ty;
    s.vy=v.y+o.w*ty+o.z*tx-o.x*tz;
    s.vz=v.z+o.w*tz+o.x*ty-o.y*tx;
    RtaRequest r;
    r.sample_dt_s=q.sample_dt_s;
    r.own_x_m=s.x; r.own_y_m=s.y; r.own_z_m=s.z;
    r.own_vx_mps=s.vx; r.own_vy_mps=s.vy; r.own_vz_mps=s.vz;
    auto& t=r.trajectory;
    t.valid=true; t.dynamically_feasible=true; t.point_count=static_cast<int>(n);
    t.peak_speed_mps=q.peak_speed_mps; t.peak_vertical_speed_mps=q.peak_vertical_speed_mps;
    t.peak_accel_mps2=q.peak_accel_mps2; t.peak_jerk_mps3=q.peak_jerk_mps3;
    t.peak_yaw_rate_dps=q.peak_yaw_rate_dps; t.peak_bank_angle_deg=q.peak_bank_angle_deg;
    for(std::size_t i=0;i<n;++i){t.trajectory_x_m[i]=q.x_m[i];t.trajectory_y_m[i]=q.y_m[i];t.trajectory_z_m[i]=q.z_m[i];}
    if(q.x_m[0]!=0 || q.y_m[0]!=0 || q.z_m[0]!=0) return;
    std::vector<RtaRequest> requests;
    for(std::size_t i=0;i<q.threat_position_m.size();i+=3){
      r.threat_rel_x_m=q.threat_position_m[i];r.threat_rel_y_m=q.threat_position_m[i+1];r.threat_rel_z_m=q.threat_position_m[i+2];
      r.relative_vx_mps=q.threat_velocity_mps[i];r.relative_vy_mps=q.threat_velocity_mps[i+1];r.relative_vz_mps=q.threat_velocity_mps[i+2];
      requests.push_back(r);
    }
    SimulationAdapter adapter(s);
    const auto result=adapter.assess_and_record(id_,q.sequence,requests,profile_,model_);
    a.recorded=result.recorded; a.reason=result.reason;
    if(a.recorded) last_accepted_=q.sequence;
    else {
      for(const auto& check : result.assessment.common_checks) {
        RCLCPP_WARN(get_logger(),
          "RTA rejection seq=%llu z=%.12g separation=%.6f terrain=%.12g traffic=%d terrain_ok=%d geofence=%d envelope=%d valid=%d",
          static_cast<unsigned long long>(q.sequence),s.z,
          check.predicted_min_separation_m,check.minimum_terrain_clearance_m,
          check.traffic_safe,check.terrain_safe,check.geofence_safe,check.envelope_safe,check.valid);
      }
    }
  }
  std::string id_;
  VehicleProfile profile_;
  ReferenceVehicleModel model_;
  std::uint64_t last_accepted_{0};
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;
  Clock::time_point received_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Service<Service>::SharedPtr service_;
};
}
int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<autonomymind::SimulationRtaNode>());
  rclcpp::shutdown();
  return 0;
}
