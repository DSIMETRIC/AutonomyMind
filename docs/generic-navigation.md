# Generic navigation core — reference ground-robot implementation

The simulator-neutral Python module `autonomymind_runtime.navigation` accepts a bounded occupancy grid, pose, mission goal, dynamic obstacle estimates and an externally constructed vehicle profile. No simulator names, scenario positions or fixed waypoint lists enter the planner.

* Range rays update known free and occupied cells. Unknown cells are blocked; occupied evidence is persistent until an external map manager replaces it.
* A* computes a four-connected route with conservative footprint inflation.
* Local unicycle controls are sampled from profile bounds. Candidate samples are checked against map occupancy and constant-velocity obstacle predictions with supplied uncertainty margins.
* Replanning recomputes routes and candidates. No route or no candidate means no motion proposal, not an authorized stop trajectory.
* C++ RTA remains independent. This module does not publish commands or claim authorization.

Reference scope: planar differential-drive robots, circular footprint, known pose and calibrated range rays in a common map frame. This is not a full SLAM, exploration, terrain, tracking, uncertainty estimation or 3D flight planner. The map uses simple persistent evidence, and local checks are sampled rather than continuous. Unknown goals are deliberately rejected; a map manager or exploration policy is needed. Dynamic obstacle estimates must come from another component. The existing Gazebo evasion demo remains a regression fixture; no live integration with the new planner yet. Profiles are Python dataclasses; configuration loading and ROS transport are separate integration tasks.

Tests: `docker compose run --build --rm test`. Seeded map tests use randomized obstacle placements and check route validity, replanning, unknown-space rejection and dynamic obstacle rejection.
