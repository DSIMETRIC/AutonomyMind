#include "autonomymind/core.hpp"

#include <cmath>
#include <iostream>
#include <limits>

using namespace autonomymind;

RawInput nominal(double timestamp) {
  RawInput input;
  input.timestamp_s = timestamp;
  input.armed = true;
  input.nav_a_valid = true;
  input.nav_b_valid = true;
  input.gnss_valid = true;
  input.vio_valid = true;
  input.nav_a_z_m = 100.0;
  input.nav_b_z_m = 100.0;
  return input;
}

int main() {
  try {
    const auto config = CoreConfig::load_strict(FM_TEST_CONFIG);

    // Disarmed input must never issue operational guidance.
    {
      Core core(config);
      auto input = nominal(0.0);
      input.armed = false;
      const auto output = core.step(input);
      if (output.decision != Decision::STANDBY ||
          output.reason != Reason::DISARMED ||
          output.guidance != Guidance::NO_COMMAND ||
          output.active_rule != ActiveRule::DISARMED) {
        return 1;
      }
    }

    // A single navigation source is deliberately de-rated; it cannot claim
    // perfect integrity merely because no independent residual is available.
    {
      Core core(config);
      auto input = nominal(0.0);
      input.nav_b_valid = false;
      const auto output = core.step(input);
      if (std::abs(output.nav_integrity - config.nav_single_source_integrity) >
              config.numeric_epsilon ||
          output.nav_integrity >= 1.0) {
        return 2;
      }
    }

    // NaN/Inf must fail closed before stateful estimators are updated.
    {
      Core core(config);
      auto input = nominal(0.0);
      input.velocity_x_mps = std::numeric_limits<double>::quiet_NaN();
      const auto output = core.step(input);
      if (output.decision != Decision::HOLD ||
          output.reason != Reason::INVALID_INPUT ||
          output.active_rule != ActiveRule::INPUT_INVALID) {
        return 3;
      }
    }

    // Time is monotonic at the Decision boundary. A regressing timestamp is
    // rejected rather than converted into a zero-dt state transition.
    {
      Core core(config);
      (void)core.step(nominal(10.0));
      const auto output = core.step(nominal(9.0));
      if (output.decision != Decision::HOLD ||
          output.reason != Reason::INVALID_INPUT) {
        return 4;
      }
    }

    // GNSS loss without a validated alternate navigation source fails closed.
    {
      Core core(config);
      auto input = nominal(0.0);
      input.gnss_valid = false;
      input.vio_valid = false;
      const auto output = core.step(input);
      if (output.decision != Decision::HOLD) {
        return 5;
      }
    }

    std::cout
        << "PASS: armed, single-source integrity, invalid input, timestamp and GPS-denied hardening\n";
    return 0;
  } catch (...) {
    return 99;
  }
}
