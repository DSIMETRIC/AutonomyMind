#pragma once
#include "autonomymind/behavior_tree.hpp"
#include "autonomymind/types.hpp"

namespace autonomymind {

struct MissionDecisionRequest {
  // Disarmed systems do not issue operational flight guidance.
  bool armed{true};

  int nav_sources_valid{};
  int nav_minimum_operational_sources{};
  int nav_sources_required{};
  double nav_integrity{};
  double nav_integrity_hold_min{};
  double nav_integrity_degrade_min{};
  bool nav_disagreement_active{false};
  bool gnss_valid{true};
  bool vio_valid{true};

  bool traffic_conflict{false};
  CollisionAvoidanceRequest collision{};

  bool mission_available{false};
  double cross_track_error_m{};
  double rejoin_enter_m{};
  double rejoin_heading_error_deg{};
  double rejoin_heading_hold_deg{};
};

struct MissionDecisionResponse {
  Decision decision{Decision::CONTINUE_MISSION};
  Reason reason{Reason::NAVIGATION_NOMINAL};
  Guidance guidance{Guidance::FOLLOW_MISSION};
  ActiveRule active_rule{ActiveRule::NONE};
  RejoinSelection selected_rejoin{RejoinSelection::NONE};
  double active_threshold_value{};
  double active_measured_value{};
  double selected_predicted_min_separation_m{};
  CollisionAvoidanceResponse avoidance{};
  BehaviorTrace trace{};
};

// Static deterministic fallback tree. Ordering is the operational priority:
// critical NAV -> traffic avoidance -> NAV verification -> degraded NAV ->
// mission rejoin -> continue mission.
class MissionDecisionTree {
 public:
  MissionDecisionResponse tick(const MissionDecisionRequest& request) const;
};

} // namespace autonomymind
