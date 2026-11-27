#include "autonomymind/mission_behavior_tree.hpp"

#include "autonomymind/gps_denied_tree.hpp"
#include "autonomymind/mission_rejoin_tree.hpp"
#include "autonomymind/nav_assurance_tree.hpp"

namespace autonomymind {

MissionDecisionResponse MissionDecisionTree::tick(
    const MissionDecisionRequest& request) const {
  MissionDecisionResponse out;

  // The mission tree is the single operational arbitration authority. Subtrees
  // evaluate their own domain, but only this ordered selector chooses a command.
  if (!request.armed) {
    out.decision = Decision::STANDBY;
    out.reason = Reason::DISARMED;
    out.guidance = Guidance::NO_COMMAND;
    out.active_rule = ActiveRule::DISARMED;
    return out;
  }

  const NavAssuranceTree nav_tree;
  const auto nav = nav_tree.tick({
      request.nav_sources_valid,
      request.nav_minimum_operational_sources,
      request.nav_sources_required,
      request.nav_integrity,
      request.nav_integrity_hold_min,
      request.nav_integrity_degrade_min,
      request.nav_disagreement_active,
  });

  // Critical NAV is evaluated before traffic by design: avoidance cannot be
  // authorized without a minimally trustworthy navigation solution.
  if (nav.severity == NavAssuranceSeverity::CRITICAL) {
    out.decision = nav.decision;
    out.reason = nav.reason;
    out.guidance = nav.guidance;
    out.active_rule = nav.rule;
    out.active_threshold_value = nav.threshold;
    out.active_measured_value = nav.measured;
    out.trace = nav.trace;
    return out;
  }

  // GPS-denied is a navigation mode concern. VIO can preserve operation, but
  // loss of GNSS without an alternate validated source fails closed.
  const GpsDeniedTree gps_tree;
  const auto gps = gps_tree.tick({
      request.gnss_valid,
      request.vio_valid,
      request.nav_sources_valid,
      request.nav_minimum_operational_sources,
      request.nav_integrity,
      request.nav_integrity_hold_min,
  });
  if (gps.active && gps.decision == Decision::HOLD) {
    out.decision = gps.decision;
    out.reason = gps.reason;
    out.guidance = gps.guidance;
    out.active_rule = gps.rule;
    out.trace = gps.trace;
    return out;
  }

  if (request.traffic_conflict) {
    const CollisionAvoidanceTree avoidance_tree;
    out.avoidance = avoidance_tree.tick(request.collision);
    out.reason = Reason::TRAFFIC_CONFLICT;
    out.active_rule = ActiveRule::TRAFFIC_CONFLICT;
    out.active_threshold_value = request.collision.min_separation_m;

    if (out.avoidance.status == BehaviorStatus::SUCCESS) {
      out.decision = out.avoidance.candidate_decision;
      out.guidance = out.avoidance.candidate_guidance;
      out.selected_predicted_min_separation_m =
          out.avoidance.selected_predicted_min_separation_m;
    } else {
      // A conflict with no Full-RTA-safe candidate must never fall through to
      // another behavior. HOLD is the deterministic fail-closed outcome.
      out.decision = Decision::HOLD;
      out.guidance = Guidance::HOLD_POSITION;
    }
    return out;
  }

  // Non-critical NAV assurance outranks mission-path recovery.
  if (nav.severity != NavAssuranceSeverity::NOMINAL) {
    out.decision = nav.decision;
    out.reason = nav.reason;
    out.guidance = nav.guidance;
    out.active_rule = nav.rule;
    out.active_threshold_value = nav.threshold;
    out.active_measured_value = nav.measured;
    out.trace = nav.trace;
    return out;
  }

  const MissionRejoinTree rejoin_tree;
  const auto rejoin = rejoin_tree.tick({
      request.mission_available,
      request.cross_track_error_m,
      request.rejoin_enter_m,
      request.rejoin_heading_error_deg,
      request.rejoin_heading_hold_deg,
  });
  if (rejoin.active) {
    out.decision = rejoin.decision;
    out.reason = rejoin.reason;
    out.guidance = rejoin.guidance;
    out.active_rule = rejoin.rule;
    out.selected_rejoin = rejoin.selected;
    out.active_threshold_value = rejoin.threshold;
    out.active_measured_value = rejoin.measured;
    out.trace = rejoin.trace;
    return out;
  }

  // No selector matched: nominal mission continuation.
  return out;
}

}  // namespace autonomymind
