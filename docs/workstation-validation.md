# Reported workstation validation

Evidence below is from operator-provided command outputs, not independently reproduced CI or certification. All experiments are separate reference runs on one Linux workstation.

| Experiment | Reported result |
|---|---|
| DDS mission replay | PASS, 3/3 synthetic tasks, 4 members, 1 reassignment, 1 revoked offer, 1 rejected completion |
| Shared World DDS | PASS, ingestion, semantic separation, provenance, expiry |
| Kinematic fleet, after planner fix d543c9f | 3/3 tasks, 97 steps, 1.76903383085207 m minimum sampled peer clearance, 0 reference guard rejections |
| Local ONNX training and export | synthetic-utility-ranker v1, SHA-256 ae8a7bf333e4bf73720ba22ff5bd716a112aea123b3820f9b092c1acf9b5cf6a |
| ONNX coordinator trace | Four model advice events, including survey reassignment; final probe metrics not supplied |
| Gazebo fleet RTA | PASS, stationary rover and drone accepted; excessive rover displacement and drone descent rejected |
| Gazebo sensor-world | PASS, 8 returns, world revision 8, changed A* route, first segment reference_checks_passed |
| Gazebo collaborative perception | PASS, revision 187, rover 9 cells, drone 8, shared 4, union 13, drone additional 4 |

The last experiment increases occupied-cell evidence relative to rover-only by 4/9 (44.4%), not total environmental coverage. Vehicles were stationary; the drone was static. No autonomous simultaneous physical motion, certified safety, LLM/VLM execution or road fleet deployment is demonstrated. Full unit-suite result was not supplied; two reported groups of four tests passed.

Single-goal collaborative planning: reported revision 1156, 9 local cells, 13 union, 4 remote additions, identical routes and zero invalidated cells. No collaborative route benefit demonstrated for this goal. Subsequent benchmark PASS: revision 170, 42 goals, 27 unchanged, 1 no shared route, 8 local unreachable, 6 alternate routes; 9 local cells, 4 remote additions. No motion or RTA assessment.

DDS reservation probe: operator-reported PASS, 3 members, 1 initial conflict, 3 scheduled and 3 revised, 0 remaining geometric conflicts, stale revision and incomplete batch rejected, expiry observed. Synthetic proposals, no C++ RTA or motion. Standalone pilot output not supplied.

Integrated stationary preview: operator-reported PASS after float64 conversion fix a8a3383. Two zero-delay reservations and two accepted C++ assessments, rover (0,0,0) to (1,0,0), drone (0,0,3) to (1,0,3), duration 2 s each. No motion; coverage incomplete.

Rover execution gate: operator-reported PASS, target_reached, displacement 0.5106645324759902 m, 12 pulses, stationary_final true, no physical hardware. Static drone; watchdog/bridge failure injection not performed.
