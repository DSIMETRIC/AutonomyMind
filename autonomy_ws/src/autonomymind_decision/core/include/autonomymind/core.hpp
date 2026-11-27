#pragma once
#include "autonomymind/config.hpp"
#include "autonomymind/types.hpp"

namespace autonomymind {

class Core {
 public:
  explicit Core(CoreConfig cfg);
  void reset();
  Output step(const RawInput& in);
  const CoreConfig& config() const { return cfg_; }

 private:
  CoreConfig cfg_;
  bool have_t0_{false};
  double t0_{};
  int align_n_{};
  double sum_ax_{},sum_ay_{},sum_az_{},sum_bx_{},sum_by_{},sum_bz_{};
  double sum_axbx_{},sum_axby_{},sum_aybx_{},sum_ayby_{};
  bool align_ready_{false};
  double align_yaw_{},align_tx_{},align_ty_{},align_tz_{};
  bool have_filter_{false};
  double filtered_residual_{};
  double bad_s_{},good_s_{};
  bool disagreement_active_{false};
  double last_t_{};
  bool have_last_t_{false};
  Decision last_decision_{Decision::CONTINUE_MISSION};
  bool state_initialized_{false};
  double state_since_s_{};
  std::uint64_t transition_count_{};
  std::uint64_t evaluation_cycle_{};
  bool have_kinematics_{false};
  double previous_speed_mps_{}, previous_accel_mps2_{};
  bool have_seen_timestamp_{false};
  double previous_input_timestamp_s_{};

  void update_alignment(const RawInput& in);
  void aligned_b(const RawInput& in,double& x,double& y,double& z) const;
  void update_nav_evidence(double t,bool valid_pair,double raw_residual);
  double clamp_unit(double v) const;
  double wrap_deg(double v) const;
};

} // namespace autonomymind
