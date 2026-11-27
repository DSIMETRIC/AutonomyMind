#pragma once
#include "autonomymind/rta.hpp"
#include <string>
#include <vector>

namespace autonomymind {
enum class VehicleKind { FixedWing, Multirotor, Rover, Generic };
struct VehicleProfile {
  std::string id;
  VehicleKind kind{VehicleKind::Generic};
  RtaConstraints limits{};
  double min_horizontal_speed_mps{};
  double max_state_age_s{};
  double max_ground_height_error_m{};
};
struct VehicleContext {
  double state_age_s{};
  bool state_valid{false};
  bool environment_valid{false};
  bool communication_valid{false};
};
struct VehicleAssessment {
  bool accepted{false};
  std::string reason;
  std::vector<RtaResult> common_checks;
};
// Extensions may impose additional constraints, never override common rejection.
class VehicleModel {
 public:
  virtual ~VehicleModel() = default;
  virtual bool supports(VehicleKind kind) const = 0;
  virtual bool envelope(const RtaRequest& request, const VehicleProfile& profile) const = 0;
};
class ReferenceVehicleModel final : public VehicleModel {
 public:
  bool supports(VehicleKind kind) const override;
  bool envelope(const RtaRequest& request, const VehicleProfile& profile) const override;
};
class VehicleRta {
 public:
  // All requests describe the same candidate, one request per observed threat.
  // Empty threat sets are denied by this initial reference API.
  VehicleAssessment evaluate(const std::vector<RtaRequest>& requests,
      const VehicleProfile& profile, const VehicleContext& context,
      const VehicleModel& model) const;
};
}
