#include "autonomymind/gps_denied_tree.hpp"

namespace autonomymind {

GpsDeniedResponse GpsDeniedTree::tick(const GpsDeniedRequest& request) const {
  GpsDeniedResponse output;

  // Root node: this subtree is only active when GNSS is unavailable.
  const int root = trace_add(
      output.trace,
      BehaviorTreeId::GPS_DENIED,
      1,
      -1,
      0,
      BehaviorNodeStatus::RUNNING);

  // If GNSS is healthy there is nothing for the GPS-denied subtree to do.
  if (request.gnss_valid) {
    trace_add(
        output.trace,
        BehaviorTreeId::GPS_DENIED,
        2,
        root,
        1,
        BehaviorNodeStatus::FAILURE);

    if (root >= 0) {
      output.trace.nodes[root].status = BehaviorNodeStatus::FAILURE;
    }
    return output;
  }

  output.active = true;

  trace_add(
      output.trace,
      BehaviorTreeId::GPS_DENIED,
      2,
      root,
      1,
      BehaviorNodeStatus::SUCCESS);

  // An alternate navigation solution is accepted only when VIO is valid,
  // the minimum number of navigation sources is available, and the computed
  // navigation integrity remains above the HOLD threshold.
  const bool alternate_navigation_available =
      request.vio_valid &&
      request.nav_sources_valid >= request.minimum &&
      request.nav_integrity >= request.hold_min;

  trace_add(
      output.trace,
      BehaviorTreeId::GPS_DENIED,
      3,
      root,
      1,
      alternate_navigation_available
          ? BehaviorNodeStatus::SUCCESS
          : BehaviorNodeStatus::FAILURE);

  if (alternate_navigation_available) {
    // GPS has been lost, but a validated alternate source exists. Continue
    // conservatively in degraded-navigation mode rather than commanding HOLD.
    output.decision = Decision::DEGRADE_NAV;
    output.reason = Reason::NAV_SOURCE_LOSS;
    output.guidance = Guidance::CONTINUE_ON_VALIDATED_SOURCE;
    output.rule = ActiveRule::GPS_DENIED;

    trace_add(
        output.trace,
        BehaviorTreeId::GPS_DENIED,
        4,
        root,
        1,
        BehaviorNodeStatus::SELECTED);
  } else {
    // Fail safe: without a validated alternate source the vehicle must not
    // continue mission execution on unassured navigation.
    output.decision = Decision::HOLD;
    output.reason = Reason::GPS_DENIED_NO_ALTERNATE;
    output.guidance = Guidance::HOLD_POSITION;
    output.rule = ActiveRule::GPS_DENIED;

    trace_add(
        output.trace,
        BehaviorTreeId::GPS_DENIED,
        5,
        root,
        1,
        BehaviorNodeStatus::SELECTED);
  }

  if (root >= 0) {
    output.trace.nodes[root].status = BehaviorNodeStatus::SUCCESS;
  }

  return output;
}

}  // namespace autonomymind
