#include "autonomymind/nav_assurance_tree.hpp"

namespace autonomymind {

NavAssuranceResponse NavAssuranceTree::tick(
    const NavAssuranceRequest& request) const {
  NavAssuranceResponse output;

  // Root node for navigation assurance. The ordering below encodes the safety
  // priority: unavailable navigation -> critical integrity -> disagreement ->
  // redundancy degradation -> integrity degradation -> nominal.
  const int root = trace_add(
      output.trace,
      BehaviorTreeId::NAV_ASSURANCE,
      1,
      -1,
      0,
      BehaviorNodeStatus::RUNNING);

  auto add_condition = [&](int node_id, bool selected) {
    return trace_add(
        output.trace,
        BehaviorTreeId::NAV_ASSURANCE,
        node_id,
        root,
        1,
        selected ? BehaviorNodeStatus::SELECTED
                 : BehaviorNodeStatus::FAILURE);
  };

  if (request.valid < request.minimum) {
    add_condition(2, true);

    output.severity = NavAssuranceSeverity::CRITICAL;
    output.decision = Decision::HOLD;
    output.reason = Reason::NAV_SOURCE_LOSS;
    output.guidance = Guidance::HOLD_POSITION;
    output.rule = ActiveRule::NAV_UNAVAILABLE;
    output.threshold = request.minimum;
    output.measured = request.valid;
  } else if (request.integrity < request.hold_min) {
    add_condition(2, false);
    add_condition(3, true);

    output.severity = NavAssuranceSeverity::CRITICAL;
    output.decision = Decision::HOLD;
    output.reason = Reason::NAV_INTEGRITY_LOW;
    output.guidance = Guidance::HOLD_POSITION;
    output.rule = ActiveRule::NAV_INTEGRITY_HOLD;
    output.threshold = request.hold_min;
    output.measured = request.integrity;
  } else if (request.disagreement) {
    add_condition(2, false);
    add_condition(3, false);
    add_condition(4, true);

    // Persistent disagreement does not immediately discard all navigation.
    // It asks the decision layer to cross-check the available sources.
    output.severity = NavAssuranceSeverity::VERIFY;
    output.decision = Decision::VERIFY_NAV;
    output.reason = Reason::NAV_SOURCE_DIVERGENCE;
    output.guidance = Guidance::CROSS_CHECK_NAV_SOURCES;
    output.rule = ActiveRule::NAV_DISAGREEMENT;
  } else if (request.valid < request.required) {
    add_condition(2, false);
    add_condition(3, false);
    add_condition(4, false);
    add_condition(5, true);

    output.severity = NavAssuranceSeverity::DEGRADED;
    output.decision = Decision::DEGRADE_NAV;
    output.reason = Reason::NAV_SOURCE_LOSS;
    output.guidance = Guidance::CONTINUE_ON_VALIDATED_SOURCE;
    output.rule = ActiveRule::NAV_REDUNDANCY_DEGRADED;
    output.threshold = request.required;
    output.measured = request.valid;
  } else if (request.integrity < request.degrade_min) {
    add_condition(2, false);
    add_condition(3, false);
    add_condition(4, false);
    add_condition(5, false);
    add_condition(6, true);

    output.severity = NavAssuranceSeverity::DEGRADED;
    output.decision = Decision::DEGRADE_NAV;
    output.reason = Reason::NAV_INTEGRITY_DEGRADED;
    output.guidance = Guidance::CONTINUE_ON_VALIDATED_SOURCE;
    output.rule = ActiveRule::NAV_INTEGRITY_DEGRADED;
    output.threshold = request.degrade_min;
    output.measured = request.integrity;
  } else {
    add_condition(2, false);
    add_condition(3, false);
    add_condition(4, false);
    add_condition(5, false);
    add_condition(6, false);

    // Navigation evidence is nominal. No corrective decision is requested.
    trace_add(
        output.trace,
        BehaviorTreeId::NAV_ASSURANCE,
        7,
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
