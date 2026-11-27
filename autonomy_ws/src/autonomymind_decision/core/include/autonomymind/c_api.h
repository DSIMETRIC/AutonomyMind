#pragma once
#include "autonomymind/generated_limits.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define FM_ABI_VERSION 2u

typedef struct fm_raw_input {
  uint32_t abi_version;
  uint32_t struct_size;
  double timestamp_s; int32_t armed;
  int32_t nav_a_valid, nav_b_valid, gnss_valid, vio_valid;
  double nav_a_x_m,nav_a_y_m,nav_a_z_m,nav_b_x_m,nav_b_y_m,nav_b_z_m;
  int32_t waypoint_count;
  double waypoint_x_m[FM_MAX_WAYPOINTS],waypoint_y_m[FM_MAX_WAYPOINTS],waypoint_z_m[FM_MAX_WAYPOINTS];
  double heading_deg;
  double velocity_x_mps,velocity_y_mps,velocity_z_mps,imu_yaw_rate_rad_s;
  int32_t threat_count;
  double own_vx_mps,own_vy_mps,own_vz_mps;
  double threat_rel_x_m[FM_MAX_THREATS],threat_rel_y_m[FM_MAX_THREATS],threat_rel_z_m[FM_MAX_THREATS];
  double threat_vx_mps[FM_MAX_THREATS],threat_vy_mps[FM_MAX_THREATS],threat_vz_mps[FM_MAX_THREATS];
} fm_raw_input;

typedef struct fm_output {
  uint32_t abi_version;
  uint32_t struct_size;
  int32_t decision,previous_decision,state_changed,reason,guidance,nav_mode,nav_health,nav_sources_valid,nav_sources_required,active_rule;
  uint32_t event_flags;
  double derived_speed_mps,derived_accel_mps2,derived_jerk_mps3,derived_vertical_speed_mps,derived_yaw_rate_dps;
  double nav_integrity,nav_residual_raw_m,nav_residual_filtered_m,nav_disagreement_bad_s,nav_recovery_good_s;
  int32_t nav_disagreement_active,alignment_ready;
  double alignment_yaw_rad,alignment_tx_m,alignment_ty_m,alignment_tz_m;
  double nav_b_aligned_x_m,nav_b_aligned_y_m,nav_b_aligned_z_m;
  double cross_track_error_m; int32_t rejoin_target_index; double rejoin_target_heading_deg,rejoin_heading_error_deg; int32_t selected_rejoin;
  int32_t active_threat_index; double no_action_predicted_min_sep_m,no_action_tcpa_s,selected_predicted_min_sep_m;
  int32_t avoidance_active,avoidance_selected_index,avoidance_safe_candidate_count;
  double avoidance_min_separation_m,avoidance_prediction_dt_s;
  int32_t avoidance_dynamically_feasible[3];
  int32_t avoidance_traffic_safe[3],avoidance_terrain_safe[3],avoidance_geofence_safe[3],avoidance_envelope_safe[3];
  int32_t avoidance_rta_safe[3],avoidance_point_count[3];
  double avoidance_predicted_min_separation_m[3],avoidance_minimum_terrain_clearance_m[3];
  double avoidance_peak_speed_mps[3],avoidance_peak_vertical_speed_mps[3];
  double avoidance_peak_accel_mps2[3],avoidance_peak_jerk_mps3[3];
  double avoidance_peak_yaw_rate_dps[3],avoidance_peak_bank_angle_deg[3];
  double avoidance_trajectory_x_m[3][FM_TRAJECTORY_POINTS];
  double avoidance_trajectory_y_m[3][FM_TRAJECTORY_POINTS];
  double avoidance_trajectory_z_m[3][FM_TRAJECTORY_POINTS];

  double time_in_state_s; uint64_t transition_count;
  double active_threshold_value,active_measured_value,transition_progress,authority_pct;
  uint64_t evaluation_cycle; int32_t evaluation_count;
  int32_t evaluation_domain[FM_EVALUATION_ROWS],evaluation_status[FM_EVALUATION_ROWS],evaluation_result[FM_EVALUATION_ROWS];
  double evaluation_measured[FM_EVALUATION_ROWS],evaluation_criterion[FM_EVALUATION_ROWS],evaluation_aux1[FM_EVALUATION_ROWS],evaluation_aux2[FM_EVALUATION_ROWS],evaluation_aux3[FM_EVALUATION_ROWS];
  char why_stays[FM_EXPLANATION_CAPACITY];
  char transition_condition[FM_EXPLANATION_CAPACITY];
  char event_summary[FM_EVENT_SUMMARY_CAPACITY];
  char config_sha256[FM_SHA256_HEX_CAPACITY];
  char config_profile[FM_PROFILE_NAME_CAPACITY];
} fm_output;

typedef void* fm_core_handle;
uint32_t fm_abi_version(void);
uint32_t fm_raw_input_struct_size(void);
uint32_t fm_output_struct_size(void);
fm_core_handle fm_core_create(const char* config_path, char* err, int err_capacity);
void fm_core_destroy(fm_core_handle h);
void fm_core_reset(fm_core_handle h);
int fm_core_step(fm_core_handle h, const fm_raw_input* in, fm_output* out, char* err, int err_capacity);
#ifdef __cplusplus
}
#endif
