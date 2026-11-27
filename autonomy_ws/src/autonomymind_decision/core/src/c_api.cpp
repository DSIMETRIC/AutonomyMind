#include "autonomymind/c_api.h"
#include "autonomymind/config.hpp"
#include "autonomymind/core.hpp"
#include <algorithm>
#include <cstring>
#include <exception>

using autonomymind::Core;using autonomymind::CoreConfig;using autonomymind::Output;using autonomymind::RawInput;
namespace {
void seterr(char* e,int n,const char* s){if(!e||n<=0)return;std::strncpy(e,s,static_cast<std::size_t>(n-1));e[n-1]='\0';}
void setstr(char* dst,int cap,const std::string& s){if(!dst||cap<=0)return;std::strncpy(dst,s.c_str(),static_cast<std::size_t>(cap-1));dst[cap-1]='\0';}
}
static_assert(RawInput::kMaxWaypoints==FM_MAX_WAYPOINTS,"C/C++ waypoint capacity mismatch");
static_assert(RawInput::kMaxThreats==FM_MAX_THREATS,"C/C++ threat capacity mismatch");
extern "C" {
uint32_t fm_abi_version(void){return FM_ABI_VERSION;}
uint32_t fm_raw_input_struct_size(void){return static_cast<uint32_t>(sizeof(fm_raw_input));}
uint32_t fm_output_struct_size(void){return static_cast<uint32_t>(sizeof(fm_output));}
fm_core_handle fm_core_create(const char* config_path,char* err,int err_capacity){if(!config_path){seterr(err,err_capacity,"null config path");return nullptr;}try{return new Core(CoreConfig::load_strict(config_path));}catch(const std::exception& e){seterr(err,err_capacity,e.what());return nullptr;}}
void fm_core_destroy(fm_core_handle h){delete static_cast<Core*>(h);}void fm_core_reset(fm_core_handle h){if(h)static_cast<Core*>(h)->reset();}
int fm_core_step(fm_core_handle h,const fm_raw_input* i,fm_output* o,char* err,int cap){
 if(!h||!i||!o){seterr(err,cap,"null argument");return -1;}
 if(i->abi_version!=FM_ABI_VERSION||i->struct_size!=sizeof(fm_raw_input)){
   seterr(err,cap,"fm_raw_input ABI version/size mismatch");return -3;
 }
 if(o->abi_version!=FM_ABI_VERSION||o->struct_size!=sizeof(fm_output)){
   seterr(err,cap,"fm_output ABI version/size mismatch");return -4;
 }
 try{
  RawInput x;x.timestamp_s=i->timestamp_s;x.armed=i->armed!=0;x.nav_a_valid=i->nav_a_valid!=0;x.nav_b_valid=i->nav_b_valid!=0;x.gnss_valid=i->gnss_valid!=0;x.vio_valid=i->vio_valid!=0;x.nav_a_x_m=i->nav_a_x_m;x.nav_a_y_m=i->nav_a_y_m;x.nav_a_z_m=i->nav_a_z_m;x.nav_b_x_m=i->nav_b_x_m;x.nav_b_y_m=i->nav_b_y_m;x.nav_b_z_m=i->nav_b_z_m;
  x.waypoint_count=std::max(0,std::min(FM_MAX_WAYPOINTS,i->waypoint_count));for(int k=0;k<x.waypoint_count;++k){x.waypoint_x_m[k]=i->waypoint_x_m[k];x.waypoint_y_m[k]=i->waypoint_y_m[k];x.waypoint_z_m[k]=i->waypoint_z_m[k];}
  x.heading_deg=i->heading_deg;x.velocity_x_mps=i->velocity_x_mps;x.velocity_y_mps=i->velocity_y_mps;x.velocity_z_mps=i->velocity_z_mps;x.imu_yaw_rate_rad_s=i->imu_yaw_rate_rad_s;
  x.threat_count=std::max(0,std::min(FM_MAX_THREATS,i->threat_count));x.own_vx_mps=i->own_vx_mps;x.own_vy_mps=i->own_vy_mps;x.own_vz_mps=i->own_vz_mps;for(int k=0;k<x.threat_count;++k){x.threat_rel_x_m[k]=i->threat_rel_x_m[k];x.threat_rel_y_m[k]=i->threat_rel_y_m[k];x.threat_rel_z_m[k]=i->threat_rel_z_m[k];x.threat_vx_mps[k]=i->threat_vx_mps[k];x.threat_vy_mps[k]=i->threat_vy_mps[k];x.threat_vz_mps[k]=i->threat_vz_mps[k];}
  const Output y=static_cast<Core*>(h)->step(x);
  o->abi_version=FM_ABI_VERSION;o->struct_size=sizeof(fm_output);
  o->decision=static_cast<int>(y.decision);o->previous_decision=static_cast<int>(y.previous_decision);o->state_changed=y.state_changed?1:0;o->reason=static_cast<int>(y.reason);o->guidance=static_cast<int>(y.guidance);o->nav_mode=static_cast<int>(y.nav_mode);o->nav_health=static_cast<int>(y.nav_health);o->nav_sources_valid=y.nav_sources_valid;o->nav_sources_required=y.nav_sources_required;o->active_rule=static_cast<int>(y.active_rule);o->event_flags=y.event_flags;
  o->derived_speed_mps=y.derived_speed_mps;o->derived_accel_mps2=y.derived_accel_mps2;o->derived_jerk_mps3=y.derived_jerk_mps3;o->derived_vertical_speed_mps=y.derived_vertical_speed_mps;o->derived_yaw_rate_dps=y.derived_yaw_rate_dps;o->nav_integrity=y.nav_integrity;o->nav_residual_raw_m=y.nav_residual_raw_m;o->nav_residual_filtered_m=y.nav_residual_filtered_m;o->nav_disagreement_bad_s=y.nav_disagreement_bad_s;o->nav_recovery_good_s=y.nav_recovery_good_s;o->nav_disagreement_active=y.nav_disagreement_active?1:0;o->alignment_ready=y.alignment_ready?1:0;o->alignment_yaw_rad=y.alignment_yaw_rad;o->alignment_tx_m=y.alignment_tx_m;o->alignment_ty_m=y.alignment_ty_m;o->alignment_tz_m=y.alignment_tz_m;o->nav_b_aligned_x_m=y.nav_b_aligned_x_m;o->nav_b_aligned_y_m=y.nav_b_aligned_y_m;o->nav_b_aligned_z_m=y.nav_b_aligned_z_m;
  o->cross_track_error_m=y.cross_track_error_m;o->rejoin_target_index=y.rejoin_target_index;o->rejoin_target_heading_deg=y.rejoin_target_heading_deg;o->rejoin_heading_error_deg=y.rejoin_heading_error_deg;o->selected_rejoin=static_cast<int>(y.selected_rejoin);o->active_threat_index=y.active_threat_index;o->no_action_predicted_min_sep_m=y.no_action_predicted_min_sep_m;o->no_action_tcpa_s=y.no_action_tcpa_s;o->selected_predicted_min_sep_m=y.selected_predicted_min_sep_m;o->avoidance_active=y.avoidance_trace.active?1:0;o->avoidance_selected_index=y.avoidance_trace.selected_index;o->avoidance_safe_candidate_count=y.avoidance_trace.safe_candidate_count;o->avoidance_min_separation_m=y.avoidance_trace.min_separation_m;o->avoidance_prediction_dt_s=y.avoidance_trace.prediction_dt_s;for(int ci=0;ci<3;++ci){o->avoidance_dynamically_feasible[ci]=y.avoidance_trace.dynamically_feasible[ci]?1:0;o->avoidance_traffic_safe[ci]=y.avoidance_trace.traffic_safe[ci]?1:0;o->avoidance_terrain_safe[ci]=y.avoidance_trace.terrain_safe[ci]?1:0;o->avoidance_geofence_safe[ci]=y.avoidance_trace.geofence_safe[ci]?1:0;o->avoidance_envelope_safe[ci]=y.avoidance_trace.envelope_safe[ci]?1:0;o->avoidance_rta_safe[ci]=y.avoidance_trace.rta_safe[ci]?1:0;o->avoidance_predicted_min_separation_m[ci]=y.avoidance_trace.predicted_min_separation_m[ci];o->avoidance_minimum_terrain_clearance_m[ci]=y.avoidance_trace.minimum_terrain_clearance_m[ci];o->avoidance_peak_speed_mps[ci]=y.avoidance_trace.peak_speed_mps[ci];o->avoidance_peak_vertical_speed_mps[ci]=y.avoidance_trace.peak_vertical_speed_mps[ci];o->avoidance_peak_accel_mps2[ci]=y.avoidance_trace.peak_accel_mps2[ci];o->avoidance_peak_jerk_mps3[ci]=y.avoidance_trace.peak_jerk_mps3[ci];o->avoidance_peak_yaw_rate_dps[ci]=y.avoidance_trace.peak_yaw_rate_dps[ci];o->avoidance_peak_bank_angle_deg[ci]=y.avoidance_trace.peak_bank_angle_deg[ci];o->avoidance_point_count[ci]=std::max(0,std::min(FM_TRAJECTORY_POINTS,y.avoidance_trace.point_count[ci]));for(int pi=0;pi<o->avoidance_point_count[ci];++pi){o->avoidance_trajectory_x_m[ci][pi]=y.avoidance_trace.trajectory_x_m[ci][pi];o->avoidance_trajectory_y_m[ci][pi]=y.avoidance_trace.trajectory_y_m[ci][pi];o->avoidance_trajectory_z_m[ci][pi]=y.avoidance_trace.trajectory_z_m[ci][pi];}}o->time_in_state_s=y.time_in_state_s;o->transition_count=y.transition_count;o->active_threshold_value=y.active_threshold_value;o->active_measured_value=y.active_measured_value;o->transition_progress=y.transition_progress;o->authority_pct=y.authority_pct;
  o->evaluation_cycle=y.evaluation_cycle;o->evaluation_count=std::max(0,std::min(FM_EVALUATION_ROWS,y.evaluation_count));for(int k=0;k<o->evaluation_count;++k){const auto& e=y.evaluations[k];o->evaluation_domain[k]=static_cast<int>(e.domain);o->evaluation_status[k]=static_cast<int>(e.status);o->evaluation_result[k]=static_cast<int>(e.result);o->evaluation_measured[k]=e.measured;o->evaluation_criterion[k]=e.criterion;o->evaluation_aux1[k]=e.aux1;o->evaluation_aux2[k]=e.aux2;o->evaluation_aux3[k]=e.aux3;}
  setstr(o->why_stays,FM_EXPLANATION_CAPACITY,y.why_stays);setstr(o->transition_condition,FM_EXPLANATION_CAPACITY,y.transition_condition);setstr(o->event_summary,FM_EVENT_SUMMARY_CAPACITY,y.event_summary);setstr(o->config_sha256,FM_SHA256_HEX_CAPACITY,y.config_sha256);setstr(o->config_profile,FM_PROFILE_NAME_CAPACITY,y.config_profile);return 0;
 }catch(const std::exception& e){seterr(err,cap,e.what());return -2;}
}
}
