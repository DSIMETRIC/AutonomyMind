#pragma once
#include "autonomymind/generated_limits.h"
#include <cstdint>
#include <string>

namespace autonomymind {

enum class Decision : std::int32_t {
  CONTINUE_MISSION=0, VERIFY_NAV=1, DEGRADE_NAV=2, REJOIN_PATH=3,
  HOLD=4, AVOID_LEFT=5, AVOID_RIGHT=6, AVOID_CLIMB=7, STANDBY=8
};
enum class Reason : std::int32_t {
  NONE=0, NAVIGATION_NOMINAL=1, NAV_SOURCE_DIVERGENCE=2,
  NAV_SOURCE_LOSS=3, NAV_INTEGRITY_LOW=4, MISSION_PATH_DEVIATION=5,
  NO_VALID_REJOIN_CANDIDATE=6, TRAFFIC_CONFLICT=7,
  CORE_CONFIG_INVALID=8, NAV_INTEGRITY_DEGRADED=9,
  DISARMED=10, INVALID_INPUT=11, GPS_DENIED_NO_ALTERNATE=12
};
enum class Guidance : std::int32_t {
  FOLLOW_MISSION=0, CROSS_CHECK_NAV_SOURCES=1,
  CONTINUE_ON_VALIDATED_SOURCE=2, HOLD_POSITION=3, TURN_LEFT=4,
  TURN_RIGHT=5, MAINTAIN_HEADING=6, CLIMB=7, NO_COMMAND=8
};
enum class NavMode : std::int32_t { UNKNOWN=0, MULTI_SOURCE=1, SINGLE_SOURCE=2, VIO_ONLY=3 };
enum class NavHealth : std::int32_t { UNKNOWN=0, NOMINAL=1, DEGRADED=2, CRITICAL=3 };
enum class ActiveRule : std::int32_t {
  NONE=0, CONFIG_INVALID=1, NAV_UNAVAILABLE=2, NAV_INTEGRITY_HOLD=3,
  TRAFFIC_CONFLICT=4, NAV_DISAGREEMENT=5, NAV_REDUNDANCY_DEGRADED=6,
  MISSION_REJOIN=7, NAV_INTEGRITY_DEGRADED=8, DISARMED=9,
  INPUT_INVALID=10, GPS_DENIED=11
};
enum class RejoinSelection : std::int32_t { NONE=0, LEFT=-1, RIGHT=1, MAINTAIN=2 };

enum class BehaviorTreeId : std::int32_t {
  MISSION_DECISION=1,
  NAV_ASSURANCE=2,
  GPS_DENIED=3,
  MISSION_REJOIN=4,
  CONTEXT_MONITOR=5,
  COLLISION_AVOIDANCE=6
};

enum class BehaviorNodeStatus : std::int32_t {
  FAILURE=0,
  SUCCESS=1,
  RUNNING=2,
  SELECTED=3
};

struct BehaviorTraceNode {
  BehaviorTreeId tree_id{BehaviorTreeId::MISSION_DECISION};
  int node_id{};
  int parent_id{-1};
  int depth{};
  BehaviorNodeStatus status{BehaviorNodeStatus::FAILURE};
  int tick_order{};
};

struct BehaviorTrace {
  int count{};
  bool overflow{false};
  BehaviorTraceNode nodes[FM_BEHAVIOR_TRACE_NODES]{};
};


enum class EvaluationDomain : std::int32_t { NAV_AVAILABILITY=0, NAV_INTEGRITY=1, NAV_DIVERGENCE=2, GPS_VIO=3, HIGH_DYNAMICS=4, VERTICAL_TRANSITION=5, MISSION=6, TRAFFIC=7, AVOIDANCE=8 };
enum class EvaluationStatus : std::int32_t { NA=0, PASS=1, ACTIVE=2, SELECTED=3 };
enum class EvaluationResult : std::int32_t { NONE=0, NOMINAL=1, HOLD_CANDIDATE=2, VERIFY_CANDIDATE=3, DEGRADE_CANDIDATE=4, VIO_ONLY=5, MULTI_SOURCE=6, SINGLE_SOURCE=7, EVENT=8, MISSION_AVAILABLE=9, REJOIN_CANDIDATE=10, TRAFFIC_CLEAR=11, TRAFFIC_CONFLICT=12, STANDBY=13, AVOID_LEFT=14, AVOID_RIGHT=15, AVOID_CLIMB=16 };

struct EvaluationRow {
  EvaluationDomain domain{EvaluationDomain::NAV_AVAILABILITY};
  EvaluationStatus status{EvaluationStatus::NA};
  EvaluationResult result{EvaluationResult::NONE};
  double measured{};
  double criterion{};
  double aux1{};
  double aux2{};
  double aux3{};
};

enum EventFlag : std::uint32_t { EVENT_NONE=0, EVENT_HIGH_DYNAMICS=1u<<0, EVENT_VERTICAL_TRANSITION=1u<<1, EVENT_NAV_DIVERGENCE=1u<<2, EVENT_TRAFFIC_CONFLICT=1u<<3 };

struct RawInput {
  double timestamp_s{};
  bool armed{true};
  bool nav_a_valid{false};
  bool nav_b_valid{false};
  bool gnss_valid{false};
  bool vio_valid{false};
  double nav_a_x_m{}, nav_a_y_m{}, nav_a_z_m{};
  double nav_b_x_m{}, nav_b_y_m{}, nav_b_z_m{};
  static constexpr int kMaxWaypoints=FM_MAX_WAYPOINTS;
  int waypoint_count{};
  double waypoint_x_m[kMaxWaypoints]{};
  double waypoint_y_m[kMaxWaypoints]{};
  double waypoint_z_m[kMaxWaypoints]{};
  double heading_deg{};
  double velocity_x_mps{}, velocity_y_mps{}, velocity_z_mps{};
  double imu_yaw_rate_rad_s{};
  static constexpr int kMaxThreats=FM_MAX_THREATS;
  int threat_count{};
  double own_vx_mps{}, own_vy_mps{}, own_vz_mps{};
  double threat_rel_x_m[kMaxThreats]{};
  double threat_rel_y_m[kMaxThreats]{};
  double threat_rel_z_m[kMaxThreats]{};
  double threat_vx_mps[kMaxThreats]{};
  double threat_vy_mps[kMaxThreats]{};
  double threat_vz_mps[kMaxThreats]{};
};

struct AvoidanceTrace {
  bool active{false};
  int selected_index{-1};
  int safe_candidate_count{};
  double min_separation_m{};
  double prediction_dt_s{};
  bool dynamically_feasible[3]{};
  bool traffic_safe[3]{};
  bool terrain_safe[3]{};
  bool geofence_safe[3]{};
  bool envelope_safe[3]{};
  bool rta_safe[3]{};
  double predicted_min_separation_m[3]{};
  double minimum_terrain_clearance_m[3]{};
  double peak_speed_mps[3]{};
  double peak_vertical_speed_mps[3]{};
  double peak_accel_mps2[3]{};
  double peak_jerk_mps3[3]{};
  double peak_yaw_rate_dps[3]{};
  double peak_bank_angle_deg[3]{};
  int point_count[3]{};
  double trajectory_x_m[3][FM_TRAJECTORY_POINTS]{};
  double trajectory_y_m[3][FM_TRAJECTORY_POINTS]{};
  double trajectory_z_m[3][FM_TRAJECTORY_POINTS]{};
};

struct MlTrace {
  bool enabled{false};
  bool inference_valid{false};
  bool used_for_selection{false};
  bool proposal_accepted{false};
  int proposed_candidate{-1};
  int selected_candidate{-1};
  double confidence{};
  double candidate_score[3]{};
  double feature_value[3][8]{};
  std::string model_name;
  std::string model_version;
  std::string model_sha256;
  std::string feature_schema;
  std::string fallback_reason;
};

struct Output {
  Decision decision{Decision::CONTINUE_MISSION};
  Decision previous_decision{Decision::CONTINUE_MISSION};
  bool state_changed{false};
  Reason reason{Reason::NONE};
  Guidance guidance{Guidance::FOLLOW_MISSION};
  NavMode nav_mode{NavMode::UNKNOWN};
  NavHealth nav_health{NavHealth::UNKNOWN};
  int nav_sources_valid{};
  int nav_sources_required{};
  ActiveRule active_rule{ActiveRule::NONE};
  std::uint32_t event_flags{EVENT_NONE};
  double derived_speed_mps{}, derived_accel_mps2{}, derived_jerk_mps3{}, derived_vertical_speed_mps{}, derived_yaw_rate_dps{};
  double nav_integrity{};
  double nav_residual_raw_m{};
  double nav_residual_filtered_m{};
  double nav_disagreement_bad_s{};
  double nav_recovery_good_s{};
  bool nav_disagreement_active{false};
  bool alignment_ready{false};
  double alignment_yaw_rad{};
  double alignment_tx_m{}, alignment_ty_m{}, alignment_tz_m{};
  double nav_b_aligned_x_m{}, nav_b_aligned_y_m{}, nav_b_aligned_z_m{};
  double cross_track_error_m{};
  int rejoin_target_index{-1};
  double rejoin_target_heading_deg{};
  double rejoin_heading_error_deg{};
  RejoinSelection selected_rejoin{RejoinSelection::NONE};
  int active_threat_index{-1};
  double no_action_predicted_min_sep_m{};
  double no_action_tcpa_s{};
  double selected_predicted_min_sep_m{};
  AvoidanceTrace avoidance_trace{};
  BehaviorTrace behavior_trace{};
  MlTrace ml_trace{};
  double time_in_state_s{};
  std::uint64_t transition_count{};
  double active_threshold_value{};
  double active_measured_value{};
  double transition_progress{};
  double authority_pct{};
  std::string why_stays;
  std::string transition_condition;
  std::string event_summary;
  std::string config_sha256;
  std::uint64_t evaluation_cycle{};
  int evaluation_count{};
  EvaluationRow evaluations[FM_EVALUATION_ROWS]{};
  std::string config_profile;
};

} // namespace autonomymind
