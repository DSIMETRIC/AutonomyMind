#pragma once
#include "autonomymind/vehicle_rta.hpp"
#include <cstdint>
#include <optional>

namespace autonomymind {
struct AdapterSnapshot {
  std::string vehicle_id;
  std::string frame_id;
  std::uint64_t sequence{};
  VehicleKind kind{VehicleKind::Generic};
  VehicleContext health{};
  double x{},y{},z{},vx{},vy{},vz{};
};
struct AdapterResult {
  bool recorded{false};
  std::string reason;
  VehicleAssessment assessment;
};
// Read-only provider contract. No transport or physical command API.
class VehicleTelemetry {
 public:
  virtual ~VehicleTelemetry() = default;
  virtual std::optional<AdapterSnapshot> snapshot() const = 0;
};
// Simulation-only recorder, deliberately has no physical output interface.
class SimulationAdapter final : public VehicleTelemetry {
 public:
  explicit SimulationAdapter(AdapterSnapshot state);
  std::optional<AdapterSnapshot> snapshot() const override;
  void update(AdapterSnapshot state);
  AdapterResult assess_and_record(const std::string& vehicle_id,
      std::uint64_t sequence, const std::vector<RtaRequest>& requests,
      const VehicleProfile& profile, const VehicleModel& model);
  const std::optional<RtaRequest>& recorded_candidate() const { return recorded_; }
 private:
  AdapterSnapshot state_;
  std::optional<RtaRequest> recorded_;
  bool consumed_{false};
};
}
