#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD="${FM_PIPELINE_BUILD:-$ROOT/.pipeline_build}"
CXX="${CXX:-g++}"
AR="${AR:-ar}"
SANITIZE="${SANITIZE:-0}"

rm -rf "$BUILD"
mkdir -p "$BUILD/generated/autonomymind" "$BUILD/bin" "$BUILD/obj"
python3 "$ROOT/tools/generate_interface_limits.py" "$ROOT/config/interface.conf" "$BUILD/generated/autonomymind"
python3 -m py_compile "$ROOT/tools/train_avoidance_ranker.py"
echo "[PASS] auditable ML trainer syntax"

FLAGS=(-std=c++17 -Wall -Wextra -Wpedantic -Werror -I"$ROOT/core/include" -I"$BUILD/generated")
if [[ "$SANITIZE" == "1" ]]; then
  FLAGS+=(-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer)
  LINK=(-fsanitize=address,undefined)
else
  FLAGS+=(-O2)
  LINK=()
fi

SOURCES=(config sha256 behavior_tree trajectory_generator rta mission_behavior_tree nav_assurance_tree gps_denied_tree mission_rejoin_tree context_monitor_tree core)
OBJECTS=()
for name in "${SOURCES[@]}"; do
  obj="$BUILD/obj/$name.o"
  "$CXX" "${FLAGS[@]}" -c "$ROOT/core/src/$name.cpp" -o "$obj"
  OBJECTS+=("$obj")
done
"$AR" rcs "$BUILD/libautonomymind_decision_core.a" "${OBJECTS[@]}"

run_test(){
  local name="$1"; shift
  "$CXX" "${FLAGS[@]}" "$@" "$ROOT/test/$name.cpp" "$BUILD/libautonomymind_decision_core.a" "${LINK[@]}" -o "$BUILD/bin/$name"
  "$BUILD/bin/$name"
}

for t in test_behavior_tree test_mission_behavior_tree test_trajectory_generator test_full_rta; do
  run_test "$t"
done

for t in test_core test_determinism test_arbitration_matrix test_threshold_boundaries test_explanation_consistency test_persistence_hysteresis test_core_invariants test_core_hardening test_ml_advisory; do
  run_test "$t" "-DFM_TEST_CONFIG=\"$ROOT/config/profiles/autonomymind_core.profile\""
done

run_test test_config "-DFM_TEST_BASELINE=\"$ROOT/config/baseline.conf\""

# Compile and exercise the public C ABI separately from the C++ core tests.
"$CXX" "${FLAGS[@]}" -fPIC -c "$ROOT/core/src/c_api.cpp" -o "$BUILD/obj/c_api.o"
"$CXX" "${FLAGS[@]}" -DFM_TEST_CONFIG="\"$ROOT/config/profiles/autonomymind_core.profile\"" \
  "$ROOT/test/test_c_api_abi.cpp" "$BUILD/obj/c_api.o" \
  "$BUILD/libautonomymind_decision_core.a" "${LINK[@]}" \
  -o "$BUILD/bin/test_c_api_abi"
"$BUILD/bin/test_c_api_abi"

# End-to-end simulated data path:
# rosbag2 SQLite/CDR -> Decision Core -> Full RTA evidence -> dashboard model.
E2E="$BUILD/e2e"
mkdir -p "$E2E/bag"
python3 "$ROOT/tools/generate_synthetic_rosbag.py" "$E2E/bag"
"$CXX" "${FLAGS[@]}" "$ROOT/tools/synthetic_replay_runner.cpp" \
  "$BUILD/libautonomymind_decision_core.a" "${LINK[@]}" \
  -o "$BUILD/bin/synthetic_replay_runner"
python3 "$ROOT/tools/replay_synthetic_rosbag.py" \
  "$E2E/bag/autonomymind_decision_0.db3" \
  "$BUILD/bin/synthetic_replay_runner" \
  "$ROOT/config/baseline.conf" \
  "$E2E/evidence.json"
node "$ROOT/test/test_dashboard_evidence.js" "$E2E/evidence.json"

if [[ "$SANITIZE" == "1" ]]; then
  echo "[PASS] AutonomyMind Decision standalone pipeline with ASan/UBSan"
else
  echo "[PASS] AutonomyMind Decision standalone pipeline"
fi
