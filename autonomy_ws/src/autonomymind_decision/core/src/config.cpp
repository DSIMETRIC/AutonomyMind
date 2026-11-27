#include "autonomymind/config.hpp"
#include "autonomymind/sha256.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace autonomymind {
namespace {
using KV=std::map<std::string,std::string>;
std::string trim(std::string s){auto notsp=[](unsigned char c){return !std::isspace(c);};s.erase(s.begin(),std::find_if(s.begin(),s.end(),notsp));s.erase(std::find_if(s.rbegin(),s.rend(),notsp).base(),s.end());return s;}
KV parse_file(const std::filesystem::path& path){
  std::ifstream f(path);if(!f)throw std::runtime_error("cannot open config: "+path.string());
  KV kv;std::string line;int ln{};
  while(std::getline(f,line)){
    ++ln;const auto hash=line.find('#');if(hash!=std::string::npos)line=line.substr(0,hash);line=trim(line);if(line.empty())continue;
    const auto eq=line.find('=');if(eq==std::string::npos)throw std::runtime_error("config line missing '=' at "+path.string()+":"+std::to_string(ln));
    const auto k=trim(line.substr(0,eq));const auto v=trim(line.substr(eq+1));if(k.empty()||v.empty())throw std::runtime_error("empty config key/value at "+path.string()+":"+std::to_string(ln));
    if(kv.count(k)){throw std::runtime_error("duplicate config key: "+k+" in "+path.string());}
    kv[k]=v;
  }
  return kv;
}
double getd(const KV& m,const char* k){auto it=m.find(k);if(it==m.end())throw std::runtime_error(std::string("missing config key: ")+k);std::size_t pos{};const double v=std::stod(it->second,&pos);if(pos!=it->second.size())throw std::runtime_error(std::string("invalid numeric config: ")+k);return v;}
int geti(const KV& m,const char* k){auto it=m.find(k);if(it==m.end())throw std::runtime_error(std::string("missing config key: ")+k);std::size_t pos{};const long v=std::stol(it->second,&pos);if(pos!=it->second.size())throw std::runtime_error(std::string("invalid integer config: ")+k);return static_cast<int>(v);}
std::string canonical(const KV& kv,const std::string& profile){std::ostringstream os;os<<"profile.name="<<profile<<'\n';for(const auto& item:kv)os<<item.first<<'='<<item.second<<'\n';return os.str();}
const std::vector<std::string>& allowed(){static const std::vector<std::string> a={
  "numeric.unit_interval_min","numeric.unit_interval_max","numeric.epsilon","geometry.pi_rad","geometry.heading_half_range_deg","geometry.heading_full_range_deg","authority.advisory_pct",
  "alignment.calibration_s","alignment.min_samples","nav.minimum_operational_sources","nav.sources_required","nav.source_b_enabled","nav.filter_tau_s","nav.verify_enter_m","nav.verify_exit_m","nav.verify_enter_s","nav.verify_exit_s","nav.integrity_full_scale_m","nav.integrity_degrade_min","nav.integrity_hold_min","nav.single_source_integrity","ml.avoidance_ranker_enabled","ml.minimum_confidence","ml.avoidance_ranker_model","mission.rejoin_enter_m","mission.rejoin_exit_m","mission.rejoin_heading_hold_deg","context.high_dynamics_speed_mps","context.high_dynamics_accel_mps2","context.high_dynamics_jerk_mps3","context.high_dynamics_yaw_rate_dps","context.vertical_transition_rate_mps","traffic.min_separation_m","traffic.prediction_horizon_s","traffic.prediction_dt_s","traffic.lateral_avoid_speed_mps","traffic.climb_avoid_speed_mps","traffic.maneuver_duration_s","traffic.turn_angle_deg","traffic.min_turn_speed_mps","physics.gravity_mps2","rta.terrain_elevation_m","rta.min_terrain_clearance_m","rta.geofence_min_x_m","rta.geofence_max_x_m","rta.geofence_min_y_m","rta.geofence_max_y_m","rta.geofence_min_z_m","rta.geofence_max_z_m","rta.max_speed_mps","rta.max_vertical_speed_mps","rta.max_accel_mps2","rta.max_jerk_mps3","rta.max_yaw_rate_dps","rta.max_bank_angle_deg","traffic.max_lateral_accel_mps2","traffic.max_vertical_accel_mps2","traffic.max_avoid_jerk_mps3"};return a;}
}

CoreConfig CoreConfig::load_strict(const std::string& input_path){
  namespace fs=std::filesystem;const fs::path p=fs::absolute(input_path);KV primary=parse_file(p);KV merged;std::string profile="baseline";fs::path baseline=p;
  if(primary.count("baseline")){
    baseline=fs::weakly_canonical(p.parent_path()/primary.at("baseline"));merged=parse_file(baseline);
    if(!primary.count("profile.name")){throw std::runtime_error("profile.name is required in profile file: "+p.string());}
    profile=primary.at("profile.name");
    primary.erase("baseline");primary.erase("profile.name");for(const auto& item:primary)merged[item.first]=item.second;
  }else{merged=primary;profile=p.stem().string();}
  const auto& a=allowed();for(const auto& item:merged)if(std::find(a.begin(),a.end(),item.first)==a.end())throw std::runtime_error("unknown config key: "+item.first);
  if(merged.size()!=a.size())throw std::runtime_error("config is incomplete: expected "+std::to_string(a.size())+" keys, got "+std::to_string(merged.size()));
  CoreConfig c;
  c.unit_interval_min=getd(merged,"numeric.unit_interval_min");c.unit_interval_max=getd(merged,"numeric.unit_interval_max");c.numeric_epsilon=getd(merged,"numeric.epsilon");c.pi_rad=getd(merged,"geometry.pi_rad");c.heading_half_range_deg=getd(merged,"geometry.heading_half_range_deg");c.heading_full_range_deg=getd(merged,"geometry.heading_full_range_deg");c.advisory_authority_pct=getd(merged,"authority.advisory_pct");
  c.alignment_calibration_s=getd(merged,"alignment.calibration_s");c.alignment_min_samples=geti(merged,"alignment.min_samples");
  c.nav_minimum_operational_sources=geti(merged,"nav.minimum_operational_sources");c.nav_sources_required=geti(merged,"nav.sources_required");c.nav_source_b_enabled=geti(merged,"nav.source_b_enabled")!=0;
  c.nav_filter_tau_s=getd(merged,"nav.filter_tau_s");c.nav_verify_enter_m=getd(merged,"nav.verify_enter_m");c.nav_verify_exit_m=getd(merged,"nav.verify_exit_m");c.nav_verify_enter_s=getd(merged,"nav.verify_enter_s");c.nav_verify_exit_s=getd(merged,"nav.verify_exit_s");c.nav_integrity_full_scale_m=getd(merged,"nav.integrity_full_scale_m");c.nav_integrity_degrade_min=getd(merged,"nav.integrity_degrade_min");c.nav_integrity_hold_min=getd(merged,"nav.integrity_hold_min");c.nav_single_source_integrity=getd(merged,"nav.single_source_integrity");c.ml_avoidance_ranker_enabled=geti(merged,"ml.avoidance_ranker_enabled")!=0;c.ml_minimum_confidence=getd(merged,"ml.minimum_confidence");c.ml_avoidance_ranker_model_path=merged.at("ml.avoidance_ranker_model");
  c.rejoin_enter_m=getd(merged,"mission.rejoin_enter_m");c.rejoin_exit_m=getd(merged,"mission.rejoin_exit_m");c.rejoin_heading_hold_deg=getd(merged,"mission.rejoin_heading_hold_deg");
  c.high_dynamics_speed_mps=getd(merged,"context.high_dynamics_speed_mps");c.high_dynamics_accel_mps2=getd(merged,"context.high_dynamics_accel_mps2");c.high_dynamics_jerk_mps3=getd(merged,"context.high_dynamics_jerk_mps3");c.high_dynamics_yaw_rate_dps=getd(merged,"context.high_dynamics_yaw_rate_dps");c.vertical_transition_rate_mps=getd(merged,"context.vertical_transition_rate_mps");
  c.traffic_min_separation_m=getd(merged,"traffic.min_separation_m");c.traffic_prediction_horizon_s=getd(merged,"traffic.prediction_horizon_s");c.traffic_prediction_dt_s=getd(merged,"traffic.prediction_dt_s");c.traffic_lateral_avoid_speed_mps=getd(merged,"traffic.lateral_avoid_speed_mps");c.traffic_climb_avoid_speed_mps=getd(merged,"traffic.climb_avoid_speed_mps");c.traffic_maneuver_duration_s=getd(merged,"traffic.maneuver_duration_s");c.traffic_turn_angle_deg=getd(merged,"traffic.turn_angle_deg");c.traffic_min_turn_speed_mps=getd(merged,"traffic.min_turn_speed_mps");c.gravity_mps2=getd(merged,"physics.gravity_mps2");c.rta_terrain_elevation_m=getd(merged,"rta.terrain_elevation_m");c.rta_min_terrain_clearance_m=getd(merged,"rta.min_terrain_clearance_m");c.rta_geofence_min_x_m=getd(merged,"rta.geofence_min_x_m");c.rta_geofence_max_x_m=getd(merged,"rta.geofence_max_x_m");c.rta_geofence_min_y_m=getd(merged,"rta.geofence_min_y_m");c.rta_geofence_max_y_m=getd(merged,"rta.geofence_max_y_m");c.rta_geofence_min_z_m=getd(merged,"rta.geofence_min_z_m");c.rta_geofence_max_z_m=getd(merged,"rta.geofence_max_z_m");c.rta_max_speed_mps=getd(merged,"rta.max_speed_mps");c.rta_max_vertical_speed_mps=getd(merged,"rta.max_vertical_speed_mps");c.rta_max_accel_mps2=getd(merged,"rta.max_accel_mps2");c.rta_max_jerk_mps3=getd(merged,"rta.max_jerk_mps3");c.rta_max_yaw_rate_dps=getd(merged,"rta.max_yaw_rate_dps");c.rta_max_bank_angle_deg=getd(merged,"rta.max_bank_angle_deg");c.traffic_max_lateral_accel_mps2=getd(merged,"traffic.max_lateral_accel_mps2");c.traffic_max_vertical_accel_mps2=getd(merged,"traffic.max_vertical_accel_mps2");c.traffic_max_avoid_jerk_mps3=getd(merged,"traffic.max_avoid_jerk_mps3");
  c.profile_name=profile;
  c.source_path=p.string();
  c.baseline_path=baseline.string();

  // Load the transparent ML ranker as data, not executable code. The model is
  // versioned, hashed and copied into CoreConfig so runtime inference performs
  // no filesystem I/O and every decision can report exact model provenance.
  const fs::path model_path =
      fs::weakly_canonical(baseline.parent_path()/c.ml_avoidance_ranker_model_path);
  const KV model=parse_file(model_path);
  const std::vector<std::string> model_keys={
      "model.name","model.version","feature.schema",
      "weight.separation","weight.terrain","weight.speed_margin",
      "weight.vertical_margin","weight.accel_margin","weight.jerk_margin",
      "weight.yaw_margin","weight.bank_margin",
      "bias.left","bias.right","bias.climb"};
  if(model.size()!=model_keys.size())throw std::runtime_error("ML model has unexpected key count");
  for(const auto& key:model_keys)if(!model.count(key))throw std::runtime_error("ML model missing key: "+key);
  c.ml_model_name=model.at("model.name");
  c.ml_model_version=model.at("model.version");
  c.ml_feature_schema=model.at("feature.schema");
  c.ml_weights={
      getd(model,"weight.separation"),
      getd(model,"weight.terrain"),
      getd(model,"weight.speed_margin"),
      getd(model,"weight.vertical_margin"),
      getd(model,"weight.accel_margin"),
      getd(model,"weight.jerk_margin"),
      getd(model,"weight.yaw_margin"),
      getd(model,"weight.bank_margin")};
  c.ml_maneuver_bias={
      getd(model,"bias.left"),
      getd(model,"bias.right"),
      getd(model,"bias.climb")};
  c.ml_model_sha256=sha256_hex(canonical(model,"ml-model"));
  c.ml_avoidance_ranker_model_path=model_path.string();

  if(c.ml_model_name.empty()||c.ml_model_version.empty()){
    throw std::runtime_error("ML model name/version must not be empty");
  }
  if(c.ml_feature_schema!="rta-safe-v1"){
    throw std::runtime_error("unsupported ML feature schema: "+c.ml_feature_schema);
  }
  for(const double weight:c.ml_weights){
    if(!std::isfinite(weight))throw std::runtime_error("ML model contains non-finite weight");
  }
  for(const double bias:c.ml_maneuver_bias){
    if(!std::isfinite(bias))throw std::runtime_error("ML model contains non-finite bias");
  }

  c.sha256=sha256_hex(canonical(merged,profile)+"ml.model.sha256="+c.ml_model_sha256+"\n");
  c.validate();
  return c;
}

void CoreConfig::validate() const{
  if(!(unit_interval_min<unit_interval_max))throw std::runtime_error("numeric unit interval invalid");
  if(!(numeric_epsilon>unit_interval_min))throw std::runtime_error("numeric.epsilon must be positive");
  if(!(pi_rad>unit_interval_max))throw std::runtime_error("geometry.pi_rad invalid");
  if(!(heading_half_range_deg>unit_interval_min&&heading_full_range_deg>heading_half_range_deg))throw std::runtime_error("heading ranges invalid");
  if(!(advisory_authority_pct>=unit_interval_min))throw std::runtime_error("authority.advisory_pct invalid");
  auto pos=[this](double x,const char* n){if(!(x>unit_interval_min))throw std::runtime_error(std::string(n)+" must be positive");};
  pos(alignment_calibration_s,"alignment.calibration_s");if(alignment_min_samples<=0)throw std::runtime_error("alignment.min_samples must be positive");
  if(nav_minimum_operational_sources<=0||nav_sources_required<nav_minimum_operational_sources)throw std::runtime_error("navigation source counts invalid");
  pos(nav_filter_tau_s,"nav.filter_tau_s");pos(nav_verify_enter_m,"nav.verify_enter_m");pos(nav_verify_exit_m,"nav.verify_exit_m");if(!(nav_verify_exit_m<nav_verify_enter_m))throw std::runtime_error("nav verify hysteresis invalid");
  pos(nav_verify_enter_s,"nav.verify_enter_s");pos(nav_verify_exit_s,"nav.verify_exit_s");pos(nav_integrity_full_scale_m,"nav.integrity_full_scale_m");
  if (!(nav_integrity_hold_min >= unit_interval_min &&
        nav_integrity_hold_min < nav_integrity_degrade_min &&
        nav_integrity_degrade_min <= unit_interval_max)) {
    throw std::runtime_error("nav integrity thresholds invalid");
  }
  if (!(nav_single_source_integrity >= nav_integrity_degrade_min &&
        nav_single_source_integrity < unit_interval_max)) {
    throw std::runtime_error(
        "nav.single_source_integrity must be degraded and below perfect integrity");
  }
  if (!(ml_minimum_confidence >= unit_interval_min &&
        ml_minimum_confidence <= unit_interval_max)) {
    throw std::runtime_error("ml.minimum_confidence must be in [0,1]");
  }
  if (ml_avoidance_ranker_model_path.empty()) {
    throw std::runtime_error("ml.avoidance_ranker_model must not be empty");
  }
  pos(rejoin_enter_m,"mission.rejoin_enter_m");if(!(rejoin_exit_m>=unit_interval_min&&rejoin_exit_m<rejoin_enter_m))throw std::runtime_error("mission rejoin hysteresis invalid");pos(rejoin_heading_hold_deg,"mission.rejoin_heading_hold_deg");
  pos(high_dynamics_speed_mps,"context.high_dynamics_speed_mps");pos(high_dynamics_accel_mps2,"context.high_dynamics_accel_mps2");pos(high_dynamics_jerk_mps3,"context.high_dynamics_jerk_mps3");pos(high_dynamics_yaw_rate_dps,"context.high_dynamics_yaw_rate_dps");pos(vertical_transition_rate_mps,"context.vertical_transition_rate_mps");
  pos(traffic_min_separation_m,"traffic.min_separation_m");pos(traffic_prediction_horizon_s,"traffic.prediction_horizon_s");pos(traffic_prediction_dt_s,"traffic.prediction_dt_s");if(traffic_prediction_dt_s>traffic_prediction_horizon_s)throw std::runtime_error("traffic prediction dt must be <= horizon");pos(traffic_lateral_avoid_speed_mps,"traffic.lateral_avoid_speed_mps");pos(traffic_climb_avoid_speed_mps,"traffic.climb_avoid_speed_mps");pos(traffic_maneuver_duration_s,"traffic.maneuver_duration_s");if(traffic_maneuver_duration_s>traffic_prediction_horizon_s)throw std::runtime_error("traffic maneuver duration must be <= horizon");pos(traffic_turn_angle_deg,"traffic.turn_angle_deg");pos(traffic_min_turn_speed_mps,"traffic.min_turn_speed_mps");pos(gravity_mps2,"physics.gravity_mps2");if(!(rta_geofence_min_x_m<rta_geofence_max_x_m&&rta_geofence_min_y_m<rta_geofence_max_y_m&&rta_geofence_min_z_m<rta_geofence_max_z_m))throw std::runtime_error("RTA geofence bounds invalid");if(rta_min_terrain_clearance_m<unit_interval_min)throw std::runtime_error("rta.min_terrain_clearance_m must be non-negative");pos(rta_max_speed_mps,"rta.max_speed_mps");pos(rta_max_vertical_speed_mps,"rta.max_vertical_speed_mps");pos(rta_max_accel_mps2,"rta.max_accel_mps2");pos(rta_max_jerk_mps3,"rta.max_jerk_mps3");pos(rta_max_yaw_rate_dps,"rta.max_yaw_rate_dps");pos(rta_max_bank_angle_deg,"rta.max_bank_angle_deg");pos(traffic_max_lateral_accel_mps2,"traffic.max_lateral_accel_mps2");pos(traffic_max_vertical_accel_mps2,"traffic.max_vertical_accel_mps2");pos(traffic_max_avoid_jerk_mps3,"traffic.max_avoid_jerk_mps3");
}
}
