#include "autonomymind/vehicle_adapter.hpp"
#include <utility>
namespace autonomymind {
SimulationAdapter::SimulationAdapter(AdapterSnapshot state):state_(std::move(state)) {}
std::optional<AdapterSnapshot> SimulationAdapter::snapshot() const { return state_; }
void SimulationAdapter::update(AdapterSnapshot state) {
  recorded_.reset();
  if(state.sequence<=state_.sequence || state.vehicle_id!=state_.vehicle_id ||
     state.kind!=state_.kind || state.frame_id!=state_.frame_id) {
    state_.health.state_valid=false;
    consumed_=true;
    return;
  }
  state_=std::move(state);
  consumed_=false;
}
AdapterResult SimulationAdapter::assess_and_record(const std::string& id,
    std::uint64_t sequence,const std::vector<RtaRequest>& requests,
    const VehicleProfile& profile,const VehicleModel& model) {
  AdapterResult out;
  recorded_.reset();
  if(consumed_ || id.empty() || id!=state_.vehicle_id || sequence!=state_.sequence ||
     state_.frame_id!="world_enu" || profile.kind!=state_.kind || requests.empty()) {
    out.reason="identity_frame_sequence_or_kind_mismatch"; return out;
  }
  for(const auto& r:requests) {
    if(r.own_x_m!=state_.x || r.own_y_m!=state_.y || r.own_z_m!=state_.z ||
       r.own_vx_mps!=state_.vx || r.own_vy_mps!=state_.vy || r.own_vz_mps!=state_.vz) {
      out.reason="candidate_state_mismatch"; return out;
    }
  }
  out.assessment=VehicleRta{}.evaluate(requests,profile,state_.health,model);
  out.reason=out.assessment.reason;
  if(out.assessment.accepted) {
    recorded_=requests.front();
    consumed_=true;
    out.recorded=true;
  }
  return out;
}
}
