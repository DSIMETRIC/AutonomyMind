#include "autonomymind/core.hpp"
#include "autonomymind/behavior_tree.hpp"
#include "autonomymind/context_monitor_tree.hpp"
#include "autonomymind/mission_behavior_tree.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
namespace autonomymind { namespace { double norm3(double x,double y,double z){return std::sqrt(x*x+y*y+z*z);} }
Core::Core(CoreConfig cfg):cfg_(std::move(cfg)){cfg_.validate();reset();}
void Core::reset(){have_t0_=false;t0_={};align_n_={};sum_ax_=sum_ay_=sum_az_=sum_bx_=sum_by_=sum_bz_={};sum_axbx_=sum_axby_=sum_aybx_=sum_ayby_={};align_ready_=false;align_yaw_=align_tx_=align_ty_=align_tz_={};have_filter_=false;filtered_residual_={};bad_s_=good_s_={};disagreement_active_=false;last_t_={};have_last_t_=false;last_decision_=Decision::CONTINUE_MISSION;state_initialized_=false;state_since_s_={};transition_count_={};evaluation_cycle_={};have_kinematics_=false;previous_speed_mps_=previous_accel_mps2_={};have_seen_timestamp_=false;previous_input_timestamp_s_={};}
double Core::clamp_unit(double v)const{return std::max(cfg_.unit_interval_min,std::min(cfg_.unit_interval_max,v));}
double Core::wrap_deg(double v)const{while(v>cfg_.heading_half_range_deg)v-=cfg_.heading_full_range_deg;while(v<-cfg_.heading_half_range_deg)v+=cfg_.heading_full_range_deg;return v;}
void Core::update_alignment(const RawInput& in){if(!(in.nav_a_valid&&in.nav_b_valid&&cfg_.nav_source_b_enabled)||align_ready_)return;if(!have_t0_){t0_=in.timestamp_s;have_t0_=true;}++align_n_;sum_ax_+=in.nav_a_x_m;sum_ay_+=in.nav_a_y_m;sum_az_+=in.nav_a_z_m;sum_bx_+=in.nav_b_x_m;sum_by_+=in.nav_b_y_m;sum_bz_+=in.nav_b_z_m;sum_axbx_+=in.nav_a_x_m*in.nav_b_x_m;sum_axby_+=in.nav_a_x_m*in.nav_b_y_m;sum_aybx_+=in.nav_a_y_m*in.nav_b_x_m;sum_ayby_+=in.nav_a_y_m*in.nav_b_y_m;if((in.timestamp_s-t0_)<cfg_.alignment_calibration_s||align_n_<cfg_.alignment_min_samples)return;const double n=static_cast<double>(align_n_),max=sum_ax_/n,may=sum_ay_/n,mbx=sum_bx_/n,mby=sum_by_/n;const double sxx=sum_axbx_-n*max*mbx,sxy=sum_axby_-n*max*mby,syx=sum_aybx_-n*may*mbx,syy=sum_ayby_-n*may*mby;align_yaw_=std::atan2(sxy-syx,sxx+syy);const double c=std::cos(align_yaw_),s=std::sin(align_yaw_);align_tx_=max-(c*mbx-s*mby);align_ty_=may-(s*mbx+c*mby);align_tz_=(sum_az_-sum_bz_)/n;align_ready_=true;}
void Core::aligned_b(const RawInput& in,double&x,double&y,double&z)const{if(!align_ready_){x=in.nav_b_x_m;y=in.nav_b_y_m;z=in.nav_b_z_m;return;}const double c=std::cos(align_yaw_),s=std::sin(align_yaw_);x=c*in.nav_b_x_m-s*in.nav_b_y_m+align_tx_;y=s*in.nav_b_x_m+c*in.nav_b_y_m+align_ty_;z=in.nav_b_z_m+align_tz_;}
void Core::update_nav_evidence(double t,bool pair,double raw){double dt{};if(have_last_t_)dt=std::max(cfg_.unit_interval_min,t-last_t_);last_t_=t;have_last_t_=true;if(pair){if(!have_filter_){filtered_residual_=raw;have_filter_=true;}else{const double a=dt>cfg_.unit_interval_min?dt/(cfg_.nav_filter_tau_s+dt):cfg_.unit_interval_min;filtered_residual_+=a*(raw-filtered_residual_);}if(filtered_residual_>=cfg_.nav_verify_enter_m){bad_s_+=dt;good_s_=0;}else if(filtered_residual_<=cfg_.nav_verify_exit_m){good_s_+=dt;bad_s_=0;}else bad_s_=good_s_=0;if(!disagreement_active_&&bad_s_>=cfg_.nav_verify_enter_s)disagreement_active_=true;if(disagreement_active_&&good_s_>=cfg_.nav_verify_exit_s)disagreement_active_=false;}else{have_filter_=false;filtered_residual_=bad_s_=good_s_=0;disagreement_active_=false;}}
Output Core::step(const RawInput& in){
 // Reject malformed or time-regressing input before it can mutate estimator
 // state. Decision fails closed instead of propagating NaN/Inf into arbitration.
 auto finite=[](double value){return std::isfinite(value);};
 bool input_valid=finite(in.timestamp_s)&&finite(in.heading_deg)&&
   finite(in.velocity_x_mps)&&finite(in.velocity_y_mps)&&finite(in.velocity_z_mps)&&
   finite(in.imu_yaw_rate_rad_s)&&finite(in.own_vx_mps)&&finite(in.own_vy_mps)&&
   finite(in.own_vz_mps)&&in.waypoint_count>=0&&
   in.waypoint_count<=RawInput::kMaxWaypoints&&in.threat_count>=0&&
   in.threat_count<=RawInput::kMaxThreats;
 if(in.nav_a_valid)input_valid=input_valid&&finite(in.nav_a_x_m)&&finite(in.nav_a_y_m)&&finite(in.nav_a_z_m);
 if(in.nav_b_valid)input_valid=input_valid&&finite(in.nav_b_x_m)&&finite(in.nav_b_y_m)&&finite(in.nav_b_z_m);
 for(int i=0;input_valid&&i<in.waypoint_count;++i)input_valid=finite(in.waypoint_x_m[i])&&finite(in.waypoint_y_m[i])&&finite(in.waypoint_z_m[i]);
 for(int i=0;input_valid&&i<in.threat_count;++i)input_valid=finite(in.threat_rel_x_m[i])&&finite(in.threat_rel_y_m[i])&&finite(in.threat_rel_z_m[i])&&finite(in.threat_vx_mps[i])&&finite(in.threat_vy_mps[i])&&finite(in.threat_vz_mps[i]);
 if(have_seen_timestamp_&&in.timestamp_s+cfg_.numeric_epsilon<previous_input_timestamp_s_)input_valid=false;
 if(!input_valid){
   Output invalid;
   invalid.evaluation_cycle=++evaluation_cycle_;
   invalid.decision=Decision::HOLD;
   invalid.previous_decision=state_initialized_?last_decision_:Decision::HOLD;
   invalid.state_changed=state_initialized_&&last_decision_!=Decision::HOLD;
   invalid.reason=Reason::INVALID_INPUT;
   invalid.guidance=Guidance::HOLD_POSITION;
   invalid.active_rule=ActiveRule::INPUT_INVALID;
   invalid.nav_health=NavHealth::CRITICAL;
   invalid.config_sha256=cfg_.sha256;
   invalid.config_profile=cfg_.profile_name;
   invalid.authority_pct=cfg_.advisory_authority_pct;
   invalid.why_stays="Invalid or non-monotonic Decision input rejected fail closed.";
   invalid.transition_condition="Provide finite, bounded and monotonic input before operational arbitration resumes.";
   invalid.event_summary="INPUT_INVALID";
   if(!state_initialized_){state_initialized_=true;state_since_s_=finite(in.timestamp_s)?in.timestamp_s:0.0;}
   else if(invalid.state_changed){++transition_count_;state_since_s_=finite(in.timestamp_s)?in.timestamp_s:state_since_s_;}
   last_decision_=Decision::HOLD;
   invalid.transition_count=transition_count_;
   return invalid;
 }
 have_seen_timestamp_=true;
 previous_input_timestamp_s_=in.timestamp_s;

 const double speed=norm3(in.velocity_x_mps,in.velocity_y_mps,in.velocity_z_mps);double accel=0,jerk=0,dt=0;if(have_kinematics_&&have_last_t_)dt=std::max(0.0,in.timestamp_s-last_t_);if(have_kinematics_&&dt>cfg_.numeric_epsilon){accel=(speed-previous_speed_mps_)/dt;jerk=(accel-previous_accel_mps2_)/dt;}previous_speed_mps_=speed;previous_accel_mps2_=accel;have_kinematics_=true;const double yaw=in.imu_yaw_rate_rad_s*cfg_.heading_half_range_deg/cfg_.pi_rad;
 update_alignment(in);double bx=in.nav_b_x_m,by=in.nav_b_y_m,bz=in.nav_b_z_m;aligned_b(in,bx,by,bz);const bool pair=in.nav_a_valid&&in.nav_b_valid&&cfg_.nav_source_b_enabled&&align_ready_;const double raw=pair?norm3(in.nav_a_x_m-bx,in.nav_a_y_m-by,in.nav_a_z_m-bz):0;update_nav_evidence(in.timestamp_s,pair,raw);
 Output o;o.evaluation_cycle=++evaluation_cycle_;o.derived_speed_mps=speed;o.derived_accel_mps2=accel;o.derived_jerk_mps3=jerk;o.derived_vertical_speed_mps=in.velocity_z_mps;o.derived_yaw_rate_dps=yaw;o.config_sha256=cfg_.sha256;o.config_profile=cfg_.profile_name;o.authority_pct=cfg_.advisory_authority_pct;o.alignment_ready=align_ready_;o.alignment_yaw_rad=align_yaw_;o.alignment_tx_m=align_tx_;o.alignment_ty_m=align_ty_;o.alignment_tz_m=align_tz_;o.nav_b_aligned_x_m=bx;o.nav_b_aligned_y_m=by;o.nav_b_aligned_z_m=bz;o.nav_residual_raw_m=raw;o.nav_residual_filtered_m=filtered_residual_;o.nav_disagreement_bad_s=bad_s_;o.nav_recovery_good_s=good_s_;o.nav_disagreement_active=disagreement_active_;
 const int valid=(in.nav_a_valid?1:0)+((cfg_.nav_source_b_enabled&&in.nav_b_valid)?1:0);o.nav_sources_valid=valid;o.nav_sources_required=cfg_.nav_sources_required;if(in.vio_valid&&!in.gnss_valid&&valid==cfg_.nav_minimum_operational_sources)o.nav_mode=NavMode::VIO_ONLY;else if(valid>=cfg_.nav_sources_required)o.nav_mode=NavMode::MULTI_SOURCE;else if(valid>=cfg_.nav_minimum_operational_sources)o.nav_mode=NavMode::SINGLE_SOURCE;o.nav_integrity=pair?clamp_unit(1-filtered_residual_/cfg_.nav_integrity_full_scale_m):(valid>=cfg_.nav_minimum_operational_sources?cfg_.nav_single_source_integrity:0);o.nav_health=(valid<cfg_.nav_minimum_operational_sources||o.nav_integrity<cfg_.nav_integrity_hold_min)?NavHealth::CRITICAL:((valid<cfg_.nav_sources_required||o.nav_integrity<cfg_.nav_integrity_degrade_min)?NavHealth::DEGRADED:NavHealth::NOMINAL);
 if(speed>=cfg_.high_dynamics_speed_mps||std::abs(accel)>=cfg_.high_dynamics_accel_mps2||std::abs(jerk)>=cfg_.high_dynamics_jerk_mps3||std::abs(yaw)>=cfg_.high_dynamics_yaw_rate_dps){o.event_flags|=EVENT_HIGH_DYNAMICS;}
 if(std::abs(in.velocity_z_mps)>=cfg_.vertical_transition_rate_mps){o.event_flags|=EVENT_VERTICAL_TRANSITION;}
 if(disagreement_active_){o.event_flags|=EVENT_NAV_DIVERGENCE;}
 double xte=0;int target=-1;double th=in.heading_deg;bool mission=false;if(in.nav_a_valid&&in.waypoint_count>=2){double best=std::numeric_limits<double>::infinity();for(int i=0;i<std::min(in.waypoint_count,RawInput::kMaxWaypoints)-1;++i){double ax=in.waypoint_x_m[i],ay=in.waypoint_y_m[i],vx=in.waypoint_x_m[i+1]-ax,vy=in.waypoint_y_m[i+1]-ay,den=vx*vx+vy*vy;if(den<=cfg_.numeric_epsilon)continue;double u=clamp_unit(((in.nav_a_x_m-ax)*vx+(in.nav_a_y_m-ay)*vy)/den),qx=ax+u*vx,qy=ay+u*vy,d=std::hypot(in.nav_a_x_m-qx,in.nav_a_y_m-qy);if(d<best){best=d;target=i+1;th=std::atan2(vy,vx)*cfg_.heading_half_range_deg/cfg_.pi_rad;}}if(std::isfinite(best)){xte=best;mission=true;}}o.cross_track_error_m=xte;o.rejoin_target_index=target;o.rejoin_target_heading_deg=th;o.rejoin_heading_error_deg=wrap_deg(th-in.heading_deg);
 double sep=std::numeric_limits<double>::infinity(),tcpa=0;int threat=-1;for(int j=0;j<std::min(in.threat_count,RawInput::kMaxThreats);++j){double rvx=in.threat_vx_mps[j]-in.own_vx_mps,rvy=in.threat_vy_mps[j]-in.own_vy_mps,rvz=in.threat_vz_mps[j]-in.own_vz_mps,vv=rvx*rvx+rvy*rvy+rvz*rvz,lt=vv>cfg_.numeric_epsilon?-(in.threat_rel_x_m[j]*rvx+in.threat_rel_y_m[j]*rvy+in.threat_rel_z_m[j]*rvz)/vv:0;lt=std::max(0.0,std::min(cfg_.traffic_prediction_horizon_s,lt));double lm=std::numeric_limits<double>::infinity();for(double t=0;t<=cfg_.traffic_prediction_horizon_s;t+=cfg_.traffic_prediction_dt_s)lm=std::min(lm,norm3(in.threat_rel_x_m[j]+rvx*t,in.threat_rel_y_m[j]+rvy*t,in.threat_rel_z_m[j]+rvz*t));if(lm<sep){sep=lm;tcpa=lt;threat=j;}}if(!std::isfinite(sep))sep=0;o.active_threat_index=threat;o.no_action_predicted_min_sep_m=sep;o.no_action_tcpa_s=tcpa;o.selected_predicted_min_sep_m=sep;
 // Build the complete collision request before mission arbitration. It is
 // evaluated only when traffic is actually conflicting.
 CollisionAvoidanceRequest collision;
 const bool traffic_conflict =
     threat >= 0 && sep < cfg_.traffic_min_separation_m;
 if (traffic_conflict) {
   o.event_flags |= EVENT_TRAFFIC_CONFLICT;
   collision.traffic_conflict = true;
   collision.rel_x_m = in.threat_rel_x_m[threat];
   collision.rel_y_m = in.threat_rel_y_m[threat];
   collision.rel_z_m = in.threat_rel_z_m[threat];
   collision.relative_vx_mps = in.threat_vx_mps[threat] - in.own_vx_mps;
   collision.relative_vy_mps = in.threat_vy_mps[threat] - in.own_vy_mps;
   collision.relative_vz_mps = in.threat_vz_mps[threat] - in.own_vz_mps;
   collision.min_separation_m = cfg_.traffic_min_separation_m;
   collision.prediction_horizon_s = cfg_.traffic_prediction_horizon_s;
   collision.prediction_dt_s = cfg_.traffic_prediction_dt_s;
   collision.climb_speed_mps = cfg_.traffic_climb_avoid_speed_mps;
   collision.turn_angle_deg = cfg_.traffic_turn_angle_deg;
   collision.min_turn_speed_mps = cfg_.traffic_min_turn_speed_mps;
   collision.gravity_mps2 = cfg_.gravity_mps2;
   if (in.nav_a_valid) {
     collision.own_x_m = in.nav_a_x_m;
     collision.own_y_m = in.nav_a_y_m;
     collision.own_z_m = in.nav_a_z_m;
   } else if (in.nav_b_valid) {
     collision.own_x_m = bx;
     collision.own_y_m = by;
     collision.own_z_m = bz;
   }
   collision.own_vx_mps = in.own_vx_mps;
   collision.own_vy_mps = in.own_vy_mps;
   collision.own_vz_mps = in.own_vz_mps;
   collision.maneuver_duration_s = cfg_.traffic_maneuver_duration_s;
   collision.terrain_elevation_m = cfg_.rta_terrain_elevation_m;
   collision.min_terrain_clearance_m = cfg_.rta_min_terrain_clearance_m;
   collision.geofence_min_x_m = cfg_.rta_geofence_min_x_m;
   collision.geofence_max_x_m = cfg_.rta_geofence_max_x_m;
   collision.geofence_min_y_m = cfg_.rta_geofence_min_y_m;
   collision.geofence_max_y_m = cfg_.rta_geofence_max_y_m;
   collision.geofence_min_z_m = cfg_.rta_geofence_min_z_m;
   collision.geofence_max_z_m = cfg_.rta_geofence_max_z_m;
   collision.max_speed_mps = cfg_.rta_max_speed_mps;
   collision.max_vertical_speed_mps = cfg_.rta_max_vertical_speed_mps;
   collision.max_accel_mps2 = cfg_.rta_max_accel_mps2;
   collision.max_jerk_mps3 = cfg_.rta_max_jerk_mps3;
   collision.max_yaw_rate_dps = cfg_.rta_max_yaw_rate_dps;
   collision.max_bank_angle_deg = cfg_.rta_max_bank_angle_deg;

   // ML is advisory only. The collision tree receives immutable model
   // coefficients plus provenance and may use them only after Full RTA PASS.
   collision.ml_ranker_enabled = cfg_.ml_avoidance_ranker_enabled;
   collision.ml_minimum_confidence = cfg_.ml_minimum_confidence;
   collision.ml_weights = cfg_.ml_weights;
   collision.ml_maneuver_bias = cfg_.ml_maneuver_bias;
   collision.ml_model_name = cfg_.ml_model_name;
   collision.ml_model_version = cfg_.ml_model_version;
   collision.ml_model_sha256 = cfg_.ml_model_sha256;
   collision.ml_feature_schema = cfg_.ml_feature_schema;
 }

 MissionDecisionRequest mission_request;
 mission_request.armed = in.armed;
 mission_request.nav_sources_valid = valid;
 mission_request.nav_minimum_operational_sources =
     cfg_.nav_minimum_operational_sources;
 mission_request.nav_sources_required = cfg_.nav_sources_required;
 mission_request.nav_integrity = o.nav_integrity;
 mission_request.nav_integrity_hold_min = cfg_.nav_integrity_hold_min;
 mission_request.nav_integrity_degrade_min = cfg_.nav_integrity_degrade_min;
 mission_request.nav_disagreement_active = disagreement_active_;
 mission_request.gnss_valid = in.gnss_valid;
 mission_request.vio_valid = in.vio_valid;
 mission_request.traffic_conflict = traffic_conflict;
 mission_request.collision = collision;
 mission_request.mission_available = mission;
 mission_request.cross_track_error_m = xte;
 mission_request.rejoin_enter_m = cfg_.rejoin_enter_m;
 mission_request.rejoin_heading_error_deg = o.rejoin_heading_error_deg;
 mission_request.rejoin_heading_hold_deg = cfg_.rejoin_heading_hold_deg;

 const MissionDecisionTree mission_tree;
 const auto authority = mission_tree.tick(mission_request);

 // Core consumes the mission tree result; it no longer independently repeats
 // the operational priority chain.
 Decision d = authority.decision;
 Reason r = authority.reason;
 Guidance g = authority.guidance;
 ActiveRule rule = authority.active_rule;
 double crit = authority.active_threshold_value;
 double meas = authority.active_measured_value;
 double prog = 1.0;
 o.selected_rejoin = authority.selected_rejoin;
 o.behavior_trace = authority.trace;

 if (traffic_conflict) {
   meas = sep;
   o.selected_predicted_min_sep_m =
       authority.selected_predicted_min_separation_m > 0.0
           ? authority.selected_predicted_min_separation_m
           : sep;

   const auto& behavior = authority.avoidance;
   o.ml_trace = behavior.ml_trace;
   o.avoidance_trace.active = true;
   o.avoidance_trace.selected_index = behavior.selected_index;
   o.avoidance_trace.safe_candidate_count = behavior.safe_candidate_count;
   o.avoidance_trace.min_separation_m = cfg_.traffic_min_separation_m;
   o.avoidance_trace.prediction_dt_s = cfg_.traffic_prediction_dt_s;

   for (int ci = 0; ci < 3; ++ci) {
     const auto& candidate =
         behavior.candidates[static_cast<std::size_t>(ci)];
     o.avoidance_trace.dynamically_feasible[ci] =
         candidate.dynamically_feasible;
     o.avoidance_trace.traffic_safe[ci] = candidate.traffic_safe;
     o.avoidance_trace.terrain_safe[ci] = candidate.terrain_safe;
     o.avoidance_trace.geofence_safe[ci] = candidate.geofence_safe;
     o.avoidance_trace.envelope_safe[ci] = candidate.envelope_safe;
     o.avoidance_trace.rta_safe[ci] = candidate.rta_safe;
     o.avoidance_trace.predicted_min_separation_m[ci] =
         candidate.predicted_min_separation_m;
     o.avoidance_trace.minimum_terrain_clearance_m[ci] =
         candidate.minimum_terrain_clearance_m;
     o.avoidance_trace.peak_speed_mps[ci] = candidate.peak_speed_mps;
     o.avoidance_trace.peak_vertical_speed_mps[ci] =
         candidate.peak_vertical_speed_mps;
     o.avoidance_trace.peak_accel_mps2[ci] = candidate.peak_accel_mps2;
     o.avoidance_trace.peak_jerk_mps3[ci] = candidate.peak_jerk_mps3;
     o.avoidance_trace.peak_yaw_rate_dps[ci] =
         candidate.peak_yaw_rate_dps;
     o.avoidance_trace.peak_bank_angle_deg[ci] =
         candidate.peak_bank_angle_deg;
     o.avoidance_trace.point_count[ci] = candidate.point_count;
     for (int pi = 0; pi < candidate.point_count; ++pi) {
       o.avoidance_trace.trajectory_x_m[ci][pi] =
           candidate.trajectory_x_m[pi];
       o.avoidance_trace.trajectory_y_m[ci][pi] =
           candidate.trajectory_y_m[pi];
       o.avoidance_trace.trajectory_z_m[ci][pi] =
           candidate.trajectory_z_m[pi];
     }
   }
 }
 o.previous_decision=state_initialized_?last_decision_:d;o.state_changed=state_initialized_&&d!=last_decision_;if(!state_initialized_){state_initialized_=true;state_since_s_=in.timestamp_s;}else if(o.state_changed){++transition_count_;state_since_s_=in.timestamp_s;}last_decision_=d;o.decision=d;o.reason=r;o.guidance=g;o.active_rule=rule;o.time_in_state_s=std::max(0.0,in.timestamp_s-state_since_s_);o.transition_count=transition_count_;o.active_threshold_value=crit;o.active_measured_value=meas;o.transition_progress=prog;
 o.evaluation_count=0;auto add=[&](EvaluationDomain dm,EvaluationStatus st,EvaluationResult rs,double m,double c,double a=0,double b=0,double z=0){auto&e=o.evaluations[o.evaluation_count++];e.domain=dm;e.status=st;e.result=rs;e.measured=m;e.criterion=c;e.aux1=a;e.aux2=b;e.aux3=z;};bool un=valid<cfg_.nav_minimum_operational_sources,red=valid<cfg_.nav_sources_required,low=o.nav_integrity<cfg_.nav_integrity_degrade_min,traf=(o.event_flags&EVENT_TRAFFIC_CONFLICT)!=0;
 add(EvaluationDomain::NAV_AVAILABILITY,(rule==ActiveRule::NAV_UNAVAILABLE||rule==ActiveRule::NAV_REDUNDANCY_DEGRADED)?EvaluationStatus::SELECTED:(un||red?EvaluationStatus::ACTIVE:EvaluationStatus::PASS),un?EvaluationResult::HOLD_CANDIDATE:(red?EvaluationResult::DEGRADE_CANDIDATE:EvaluationResult::NOMINAL),valid,cfg_.nav_sources_required,cfg_.nav_minimum_operational_sources);
 add(EvaluationDomain::NAV_INTEGRITY,(rule==ActiveRule::NAV_INTEGRITY_HOLD||rule==ActiveRule::NAV_INTEGRITY_DEGRADED)?EvaluationStatus::SELECTED:(low?EvaluationStatus::ACTIVE:EvaluationStatus::PASS),rule==ActiveRule::NAV_INTEGRITY_HOLD?EvaluationResult::HOLD_CANDIDATE:(rule==ActiveRule::NAV_INTEGRITY_DEGRADED?EvaluationResult::DEGRADE_CANDIDATE:EvaluationResult::NOMINAL),o.nav_integrity,cfg_.nav_integrity_hold_min,cfg_.nav_integrity_degrade_min);
 add(EvaluationDomain::NAV_DIVERGENCE,!pair?EvaluationStatus::NA:(rule==ActiveRule::NAV_DISAGREEMENT?EvaluationStatus::SELECTED:(disagreement_active_?EvaluationStatus::ACTIVE:EvaluationStatus::PASS)),disagreement_active_?EvaluationResult::VERIFY_CANDIDATE:EvaluationResult::NOMINAL,filtered_residual_,cfg_.nav_verify_enter_m,bad_s_,cfg_.nav_verify_enter_s,cfg_.nav_verify_exit_m);
 EvaluationResult sr=o.nav_mode==NavMode::VIO_ONLY?EvaluationResult::VIO_ONLY:(o.nav_mode==NavMode::MULTI_SOURCE?EvaluationResult::MULTI_SOURCE:(o.nav_mode==NavMode::SINGLE_SOURCE?EvaluationResult::SINGLE_SOURCE:EvaluationResult::NONE));add(EvaluationDomain::GPS_VIO,EvaluationStatus::PASS,sr,(int)o.nav_mode,cfg_.nav_sources_required,in.gnss_valid,in.vio_valid);
 add(EvaluationDomain::HIGH_DYNAMICS,(o.event_flags&EVENT_HIGH_DYNAMICS)?EvaluationStatus::ACTIVE:EvaluationStatus::PASS,(o.event_flags&EVENT_HIGH_DYNAMICS)?EvaluationResult::EVENT:EvaluationResult::NOMINAL,speed,cfg_.high_dynamics_speed_mps,accel,jerk,yaw);
 add(EvaluationDomain::VERTICAL_TRANSITION,(o.event_flags&EVENT_VERTICAL_TRANSITION)?EvaluationStatus::ACTIVE:EvaluationStatus::PASS,(o.event_flags&EVENT_VERTICAL_TRANSITION)?EvaluationResult::EVENT:EvaluationResult::NOMINAL,std::abs(in.velocity_z_mps),cfg_.vertical_transition_rate_mps);
 add(EvaluationDomain::MISSION,!mission?EvaluationStatus::NA:(rule==ActiveRule::MISSION_REJOIN?EvaluationStatus::SELECTED:(xte>=cfg_.rejoin_enter_m?EvaluationStatus::ACTIVE:EvaluationStatus::PASS)),mission?(xte>=cfg_.rejoin_enter_m?EvaluationResult::REJOIN_CANDIDATE:EvaluationResult::MISSION_AVAILABLE):EvaluationResult::NONE,xte,cfg_.rejoin_enter_m);
 add(EvaluationDomain::TRAFFIC,threat<0?EvaluationStatus::NA:(rule==ActiveRule::TRAFFIC_CONFLICT?EvaluationStatus::SELECTED:(traf?EvaluationStatus::ACTIVE:EvaluationStatus::PASS)),traf?EvaluationResult::TRAFFIC_CONFLICT:(threat>=0?EvaluationResult::TRAFFIC_CLEAR:EvaluationResult::NONE),sep,cfg_.traffic_min_separation_m,tcpa,threat);
 add(EvaluationDomain::AVOIDANCE,threat<0?EvaluationStatus::NA:(rule==ActiveRule::TRAFFIC_CONFLICT?EvaluationStatus::SELECTED:EvaluationStatus::PASS),d==Decision::AVOID_LEFT?EvaluationResult::AVOID_LEFT:(d==Decision::AVOID_RIGHT?EvaluationResult::AVOID_RIGHT:(d==Decision::AVOID_CLIMB?EvaluationResult::AVOID_CLIMB:EvaluationResult::STANDBY)),sep,cfg_.traffic_min_separation_m);
 std::ostringstream ev;if(o.event_flags==0)ev<<"NONE";else{if(o.event_flags&EVENT_NAV_DIVERGENCE)ev<<"NAV_DIVERGENCE ";if(o.event_flags&EVENT_HIGH_DYNAMICS)ev<<"HIGH_DYNAMICS ";if(o.event_flags&EVENT_VERTICAL_TRANSITION)ev<<"VERTICAL_TRANSITION ";if(o.event_flags&EVENT_TRAFFIC_CONFLICT)ev<<"TRAFFIC_CONFLICT";}o.event_summary=ev.str();std::ostringstream why,next;why<<"Decision "<<static_cast<int>(d)<<" selected by Core rule "<<static_cast<int>(rule)<<" with measured value "<<meas<<" and criterion "<<crit<<".";next<<"State changes only when a configured higher-priority or recovery rule becomes active.";o.why_stays=why.str();o.transition_condition=next.str();return o;
}}
