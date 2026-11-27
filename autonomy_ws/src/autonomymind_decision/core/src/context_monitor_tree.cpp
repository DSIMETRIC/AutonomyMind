#include "autonomymind/context_monitor_tree.hpp"

#include <cmath>

namespace autonomymind {

ContextMonitorResponse ContextMonitorTree::tick(
    const ContextMonitorRequest& request) const {
  ContextMonitorResponse output;

  const int root = trace_add(
      output.trace,
      BehaviorTreeId::CONTEXT_MONITOR,
      1,
      -1,
      0,
      BehaviorNodeStatus::RUNNING);

  // High-dynamics context is informational evidence used by Decision/HMI. It
  // becomes active when any configured translational or rotational threshold
  // is crossed.
  const bool high_dynamics =
      request.speed >= request.high_speed ||
      std::abs(request.accel) >= request.high_accel ||
      std::abs(request.jerk) >= request.high_jerk ||
      std::abs(request.yaw_rate_dps) >= request.high_yaw;

  // Vertical-transition context is tracked independently so climb/descent can
  // be explained even when the other dynamics remain nominal.
  const bool vertical_transition =
      std::abs(request.vertical_speed) >= request.vertical_rate;

  trace_add(
      output.trace,
      BehaviorTreeId::CONTEXT_MONITOR,
      2,
      root,
      1,
      high_dynamics ? BehaviorNodeStatus::SUCCESS
                    : BehaviorNodeStatus::FAILURE);

  trace_add(
      output.trace,
      BehaviorTreeId::CONTEXT_MONITOR,
      3,
      root,
      1,
      vertical_transition ? BehaviorNodeStatus::SUCCESS
                          : BehaviorNodeStatus::FAILURE);

  if (high_dynamics) {
    output.event_flags |= EVENT_HIGH_DYNAMICS;
  }

  if (vertical_transition) {
    output.event_flags |= EVENT_VERTICAL_TRANSITION;
  }

  if (root >= 0) {
    output.trace.nodes[root].status = BehaviorNodeStatus::SUCCESS;
  }

  return output;
}

}  // namespace autonomymind
