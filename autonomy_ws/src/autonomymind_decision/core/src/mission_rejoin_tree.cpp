#include "autonomymind/mission_rejoin_tree.hpp"

#include <cmath>

namespace autonomymind {

MissionRejoinResponse MissionRejoinTree::tick(
    const MissionRejoinRequest& request) const {
  MissionRejoinResponse output;

  const int root = trace_add(
      output.trace,
      BehaviorTreeId::MISSION_REJOIN,
      1,
      -1,
      0,
      BehaviorNodeStatus::RUNNING);

  // Rejoin is only considered when a mission path exists and cross-track error
  // has crossed the configured enter threshold.
  const bool path_deviation =
      request.mission_available && request.xte_m >= request.enter_m;

  trace_add(
      output.trace,
      BehaviorTreeId::MISSION_REJOIN,
      2,
      root,
      1,
      path_deviation ? BehaviorNodeStatus::SUCCESS
                     : BehaviorNodeStatus::FAILURE);

  if (!path_deviation) {
    if (root >= 0) {
      output.trace.nodes[root].status = BehaviorNodeStatus::FAILURE;
    }
    return output;
  }

  output.active = true;
  output.decision = Decision::REJOIN_PATH;
  output.reason = Reason::MISSION_PATH_DEVIATION;
  output.rule = ActiveRule::MISSION_REJOIN;
  output.threshold = request.enter_m;
  output.measured = request.xte_m;

  // Heading error decides which rejoin guidance is proposed. A small heading
  // error is intentionally held to avoid unnecessary left/right oscillation.
  if (std::abs(request.heading_error_deg) <= request.heading_hold_deg) {
    output.guidance = Guidance::MAINTAIN_HEADING;
    output.selected = RejoinSelection::MAINTAIN;
    trace_add(
        output.trace,
        BehaviorTreeId::MISSION_REJOIN,
        5,
        root,
        1,
        BehaviorNodeStatus::SELECTED);
  } else if (request.heading_error_deg > 0.0) {
    output.guidance = Guidance::TURN_LEFT;
    output.selected = RejoinSelection::LEFT;
    trace_add(
        output.trace,
        BehaviorTreeId::MISSION_REJOIN,
        3,
        root,
        1,
        BehaviorNodeStatus::SELECTED);
  } else {
    output.guidance = Guidance::TURN_RIGHT;
    output.selected = RejoinSelection::RIGHT;
    trace_add(
        output.trace,
        BehaviorTreeId::MISSION_REJOIN,
        4,
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
