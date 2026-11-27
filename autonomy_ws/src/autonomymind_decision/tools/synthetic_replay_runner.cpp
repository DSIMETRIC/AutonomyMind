#include "autonomymind/core.hpp"
#include "autonomymind/config.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using autonomymind::Decision;
using autonomymind::Output;
using autonomymind::RawInput;

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string item;
  while (std::getline(stream, item, ',')) {
    fields.push_back(item);
  }
  return fields;
}

double asDouble(const std::string& value) {
  return std::stod(value);
}

int asInt(const std::string& value) {
  return std::stoi(value);
}

const char* decisionName(Decision decision) {
  switch (decision) {
    case Decision::CONTINUE_MISSION: return "CONTINUE_MISSION";
    case Decision::VERIFY_NAV: return "VERIFY_NAV";
    case Decision::DEGRADE_NAV: return "DEGRADE_NAV";
    case Decision::REJOIN_PATH: return "REJOIN_PATH";
    case Decision::HOLD: return "HOLD";
    case Decision::AVOID_LEFT: return "AVOID_LEFT";
    case Decision::AVOID_RIGHT: return "AVOID_RIGHT";
    case Decision::AVOID_CLIMB: return "AVOID_CLIMB";
    case Decision::STANDBY: return "STANDBY";
  }
  return "UNKNOWN";
}

struct Stats {
  int samples{};
  int continue_count{};
  int degrade_count{};
  int rejoin_count{};
  int hold_count{};
  int avoid_count{};
  int traffic_samples{};
  int selected_full_rta_samples{};
  bool saw_gps_denied{};
  bool saw_traffic{};
  bool saw_rejoin{};
  Output last_avoidance{};
  bool have_last_avoidance{};
};

void countDecision(Stats& stats, const Output& output) {
  ++stats.samples;
  switch (output.decision) {
    case Decision::CONTINUE_MISSION:
      ++stats.continue_count;
      break;
    case Decision::DEGRADE_NAV:
      ++stats.degrade_count;
      break;
    case Decision::REJOIN_PATH:
      ++stats.rejoin_count;
      stats.saw_rejoin = true;
      break;
    case Decision::HOLD:
      ++stats.hold_count;
      break;
    case Decision::AVOID_LEFT:
    case Decision::AVOID_RIGHT:
    case Decision::AVOID_CLIMB:
      ++stats.avoid_count;
      stats.saw_traffic = true;
      stats.last_avoidance = output;
      stats.have_last_avoidance = true;
      if (output.avoidance_trace.selected_index >= 0) {
        const int index = output.avoidance_trace.selected_index;
        if (output.avoidance_trace.rta_safe[index] &&
            output.avoidance_trace.traffic_safe[index] &&
            output.avoidance_trace.terrain_safe[index] &&
            output.avoidance_trace.geofence_safe[index] &&
            output.avoidance_trace.envelope_safe[index]) {
          ++stats.selected_full_rta_samples;
        }
      }
      break;
    default:
      break;
  }

  if ((output.event_flags & autonomymind::EVENT_TRAFFIC_CONFLICT) != 0u) {
    ++stats.traffic_samples;
  }
}

void writeCandidate(
    std::ostream& output,
    const autonomymind::AvoidanceTrace& trace,
    int index,
    const char* name) {
  output << "    {\n";
  output << "      \"name\": \"" << name << "\",\n";
  output << "      \"dynamically_feasible\": "
         << (trace.dynamically_feasible[index] ? "true" : "false") << ",\n";
  output << "      \"traffic_safe\": "
         << (trace.traffic_safe[index] ? "true" : "false") << ",\n";
  output << "      \"terrain_safe\": "
         << (trace.terrain_safe[index] ? "true" : "false") << ",\n";
  output << "      \"geofence_safe\": "
         << (trace.geofence_safe[index] ? "true" : "false") << ",\n";
  output << "      \"envelope_safe\": "
         << (trace.envelope_safe[index] ? "true" : "false") << ",\n";
  output << "      \"rta_safe\": "
         << (trace.rta_safe[index] ? "true" : "false") << ",\n";
  output << "      \"predicted_min_separation_m\": "
         << trace.predicted_min_separation_m[index] << ",\n";
  output << "      \"minimum_terrain_clearance_m\": "
         << trace.minimum_terrain_clearance_m[index] << ",\n";
  output << "      \"peak_speed_mps\": "
         << trace.peak_speed_mps[index] << ",\n";
  output << "      \"peak_vertical_speed_mps\": "
         << trace.peak_vertical_speed_mps[index] << ",\n";
  output << "      \"peak_accel_mps2\": "
         << trace.peak_accel_mps2[index] << ",\n";
  output << "      \"peak_jerk_mps3\": "
         << trace.peak_jerk_mps3[index] << ",\n";
  output << "      \"peak_yaw_rate_dps\": "
         << trace.peak_yaw_rate_dps[index] << ",\n";
  output << "      \"peak_bank_angle_deg\": "
         << trace.peak_bank_angle_deg[index] << "\n";
  output << "    }";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: synthetic_replay_runner CONFIG OUTPUT_JSON\n";
    return 2;
  }

  const auto config = autonomymind::CoreConfig::load_strict(argv[1]);
  autonomymind::Core core(config);

  Stats stats;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) {
      continue;
    }

    const auto values = splitCsv(line);
    if (values.size() != 20u) {
      std::cerr << "bad synthetic row: expected 20 fields, got "
                << values.size() << "\n";
      return 3;
    }

    RawInput input;
    std::size_t i = 0;
    input.timestamp_s = asDouble(values[i++]);
    input.armed = asInt(values[i++]) != 0;
    input.nav_a_valid = asInt(values[i++]) != 0;
    input.nav_b_valid = asInt(values[i++]) != 0;
    input.gnss_valid = asInt(values[i++]) != 0;
    input.vio_valid = asInt(values[i++]) != 0;

    const double nav_x_m = asDouble(values[i++]);
    const double nav_y_m = asDouble(values[i++]);
    const double nav_z_m = asDouble(values[i++]);

    input.nav_a_x_m = nav_x_m;
    input.nav_a_y_m = nav_y_m;
    input.nav_a_z_m = nav_z_m;
    input.nav_b_x_m = nav_x_m;
    input.nav_b_y_m = nav_y_m;
    input.nav_b_z_m = nav_z_m;

    input.heading_deg = asDouble(values[i++]);
    input.velocity_x_mps = asDouble(values[i++]);
    input.velocity_y_mps = asDouble(values[i++]);
    input.velocity_z_mps = asDouble(values[i++]);
    input.own_vx_mps = input.velocity_x_mps;
    input.own_vy_mps = input.velocity_y_mps;
    input.own_vz_mps = input.velocity_z_mps;

    input.threat_count = asInt(values[i++]);
    if (input.threat_count > 0) {
      input.threat_rel_x_m[0] = asDouble(values[i++]);
      input.threat_rel_y_m[0] = asDouble(values[i++]);
      input.threat_rel_z_m[0] = asDouble(values[i++]);
      input.threat_vx_mps[0] = asDouble(values[i++]);
      input.threat_vy_mps[0] = asDouble(values[i++]);
      input.threat_vz_mps[0] = asDouble(values[i++]);
    } else {
      i += 6;
    }

    // A straight mission along +X at y=0 provides a deterministic path for
    // the rejoin segment after traffic has cleared.
    input.waypoint_count = 2;
    input.waypoint_x_m[0] = 0.0;
    input.waypoint_y_m[0] = 0.0;
    input.waypoint_z_m[0] = 100.0;
    input.waypoint_x_m[1] = 200.0;
    input.waypoint_y_m[1] = 0.0;
    input.waypoint_z_m[1] = 100.0;

    if (!input.gnss_valid) {
      stats.saw_gps_denied = true;
    }

    const auto output = core.step(input);
    countDecision(stats, output);
  }

  if (stats.samples < 50 ||
      !stats.saw_gps_denied ||
      !stats.saw_traffic ||
      !stats.saw_rejoin ||
      stats.selected_full_rta_samples <= 0) {
    std::cerr << "synthetic temporal scenario did not exercise required states\n";
    return 4;
  }

  std::ofstream json(argv[2]);
  if (!json) {
    std::cerr << "cannot create dashboard evidence: " << argv[2] << "\n";
    return 5;
  }

  json << std::fixed << std::setprecision(6);
  json << "{\n";
  json << "  \"samples\": " << stats.samples << ",\n";
  json << "  \"continue_count\": " << stats.continue_count << ",\n";
  json << "  \"degrade_count\": " << stats.degrade_count << ",\n";
  json << "  \"rejoin_count\": " << stats.rejoin_count << ",\n";
  json << "  \"hold_count\": " << stats.hold_count << ",\n";
  json << "  \"avoid_count\": " << stats.avoid_count << ",\n";
  json << "  \"traffic_samples\": " << stats.traffic_samples << ",\n";
  json << "  \"selected_full_rta_samples\": "
       << stats.selected_full_rta_samples << ",\n";

  if (stats.have_last_avoidance) {
    const auto& output = stats.last_avoidance;
    json << "  \"decision\": \"" << decisionName(output.decision) << "\",\n";
    json << "  \"selected_index\": "
         << output.avoidance_trace.selected_index << ",\n";
    json << "  \"ml\": {\n";
    json << "    \"enabled\": " << (output.ml_trace.enabled ? "true" : "false") << ",\n";
    json << "    \"inference_valid\": " << (output.ml_trace.inference_valid ? "true" : "false") << ",\n";
    json << "    \"used_for_selection\": " << (output.ml_trace.used_for_selection ? "true" : "false") << ",\n";
    json << "    \"proposal_accepted\": " << (output.ml_trace.proposal_accepted ? "true" : "false") << ",\n";
    json << "    \"proposed_candidate\": " << output.ml_trace.proposed_candidate << ",\n";
    json << "    \"selected_candidate\": " << output.ml_trace.selected_candidate << ",\n";
    json << "    \"confidence\": " << output.ml_trace.confidence << ",\n";
    json << "    \"model_name\": \"" << output.ml_trace.model_name << "\",\n";
    json << "    \"model_version\": \"" << output.ml_trace.model_version << "\",\n";
    json << "    \"model_sha256\": \"" << output.ml_trace.model_sha256 << "\",\n";
    json << "    \"feature_schema\": \"" << output.ml_trace.feature_schema << "\",\n";
    json << "    \"fallback_reason\": \"" << output.ml_trace.fallback_reason << "\",\n";
    json << "    \"candidate_scores\": ["
         << output.ml_trace.candidate_score[0] << ","
         << output.ml_trace.candidate_score[1] << ","
         << output.ml_trace.candidate_score[2] << "],\n";
    json << "    \"features\": [\n";
    for (int ci = 0; ci < 3; ++ci) {
      json << "      [";
      for (int fi = 0; fi < 8; ++fi) {
        if (fi > 0) json << ",";
        json << output.ml_trace.feature_value[ci][fi];
      }
      json << "]" << (ci < 2 ? "," : "") << "\n";
    }
    json << "    ]\n";
    json << "  },\n";
    json << "  \"candidates\": [\n";
    writeCandidate(json, output.avoidance_trace, 0, "LEFT");
    json << ",\n";
    writeCandidate(json, output.avoidance_trace, 1, "RIGHT");
    json << ",\n";
    writeCandidate(json, output.avoidance_trace, 2, "CLIMB");
    json << "\n  ]\n";
  } else {
    json << "  \"decision\": \"NONE\",\n";
    json << "  \"selected_index\": -1,\n";
    json << "  \"candidates\": []\n";
  }

  json << "}\n";

  std::cout << "[PASS] temporal simulated Decision replay: "
            << stats.samples << " samples, "
            << stats.selected_full_rta_samples
            << " fully assured avoidance samples\n";

  if (stats.have_last_avoidance) {
    const auto& ml = stats.last_avoidance.ml_trace;
    std::cout << "[PASS] auditable ML evidence: "
              << ml.model_name << "@" << ml.model_version
              << " sha256=" << ml.model_sha256
              << " confidence=" << ml.confidence
              << " used=" << (ml.used_for_selection ? "yes" : "no")
              << " proposed=" << ml.proposed_candidate
              << " selected=" << ml.selected_candidate << "\n";
  }
  return 0;
}
