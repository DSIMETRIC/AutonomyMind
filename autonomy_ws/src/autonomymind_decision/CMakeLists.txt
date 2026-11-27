cmake_minimum_required(VERSION 3.16)
project(autonomymind_decision VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/autonomymind")
set(GEN_HDR "${GEN_DIR}/generated_limits.h")
add_custom_command(
  OUTPUT "${GEN_HDR}"
  COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_interface_limits.py"
          "${CMAKE_CURRENT_SOURCE_DIR}/config/interface.conf" "${GEN_DIR}"
  DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/config/interface.conf"
          "${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_interface_limits.py"
  VERBATIM)
add_custom_target(generate_interface_limits DEPENDS "${GEN_HDR}")

add_library(autonomymind_decision_core SHARED
  core/src/config.cpp
  core/src/sha256.cpp
  core/src/behavior_tree.cpp
  core/src/trajectory_generator.cpp
  core/src/rta.cpp
  core/src/vehicle_rta.cpp
  core/src/vehicle_adapter.cpp
  core/src/mission_behavior_tree.cpp
  core/src/nav_assurance_tree.cpp
  core/src/gps_denied_tree.cpp
  core/src/mission_rejoin_tree.cpp
  core/src/context_monitor_tree.cpp
  core/src/core.cpp
  "${GEN_HDR}")
add_dependencies(autonomymind_decision_core generate_interface_limits)
target_include_directories(autonomymind_decision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
  $<INSTALL_INTERFACE:include>)
target_compile_options(autonomymind_decision_core PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_library(autonomymind_decision_c_api SHARED core/src/c_api.cpp)
add_dependencies(autonomymind_decision_c_api generate_interface_limits)
target_link_libraries(autonomymind_decision_c_api PRIVATE autonomymind_decision_core)
target_include_directories(autonomymind_decision_c_api PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>
  $<INSTALL_INTERFACE:include>)
target_compile_options(autonomymind_decision_c_api PRIVATE -Wall -Wextra -Wpedantic -Werror)
set_target_properties(autonomymind_decision_c_api PROPERTIES OUTPUT_NAME autonomymind_decision)

install(TARGETS autonomymind_decision_core autonomymind_decision_c_api
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY core/include/ DESTINATION include)
install(FILES "${GEN_HDR}" DESTINATION include/autonomymind)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)

if(BUILD_TESTING)
  function(add_fm_decision_test target source)
    add_executable(${target} ${source})
    add_dependencies(${target} generate_interface_limits)
    target_link_libraries(${target} PRIVATE autonomymind_decision_core)
    target_include_directories(${target} PRIVATE core/include "${CMAKE_CURRENT_BINARY_DIR}/generated")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_test(NAME ${target} COMMAND ${target})
  endfunction()
  enable_testing()
  add_fm_decision_test(test_behavior_tree test/test_behavior_tree.cpp)
  add_fm_decision_test(test_mission_behavior_tree test/test_mission_behavior_tree.cpp)
  add_fm_decision_test(test_trajectory_generator test/test_trajectory_generator.cpp)
  add_fm_decision_test(test_full_rta test/test_full_rta.cpp)
  add_fm_decision_test(test_vehicle_rta test/test_vehicle_rta.cpp)
  add_fm_decision_test(test_vehicle_adapter test/test_vehicle_adapter.cpp)
  add_fm_decision_test(test_ml_advisory test/test_ml_advisory.cpp)
  target_compile_definitions(test_ml_advisory PRIVATE FM_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/profiles/autonomymind_core.profile")
  add_fm_decision_test(test_core test/test_core.cpp)
  target_compile_definitions(test_core PRIVATE FM_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/profiles/autonomymind_core.profile")
  add_fm_decision_test(test_config test/test_config.cpp)
  target_compile_definitions(test_config PRIVATE FM_TEST_BASELINE="${CMAKE_CURRENT_SOURCE_DIR}/config/baseline.conf")
  add_fm_decision_test(test_determinism test/test_determinism.cpp)
  target_compile_definitions(test_determinism PRIVATE FM_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/profiles/autonomymind_core.profile")
  add_fm_decision_test(test_arbitration_matrix test/test_arbitration_matrix.cpp)
  target_compile_definitions(test_arbitration_matrix PRIVATE FM_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/profiles/autonomymind_core.profile")
  add_fm_decision_test(test_threshold_boundaries test/test_threshold_boundaries.cpp)
  target_compile_definitions(test_threshold_boundaries PRIVATE FM_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/profiles/autonomymind_core.profile")
  add_fm_decision_test(test_explanation_consistency test/test_explanation_consistency.cpp)
  target_compile_definitions(test_explanation_consistency PRIVATE FM_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/profiles/autonomymind_core.profile")
  add_fm_decision_test(test_persistence_hysteresis test/test_persistence_hysteresis.cpp)
  target_compile_definitions(test_persistence_hysteresis PRIVATE FM_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/profiles/autonomymind_core.profile")
  add_fm_decision_test(test_core_invariants test/test_core_invariants.cpp)
  target_compile_definitions(test_core_invariants PRIVATE FM_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/profiles/autonomymind_core.profile")
  add_fm_decision_test(test_core_hardening test/test_core_hardening.cpp)
  target_compile_definitions(test_core_hardening PRIVATE FM_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/profiles/autonomymind_core.profile")
  add_executable(test_c_api_abi test/test_c_api_abi.cpp)
  add_dependencies(test_c_api_abi generate_interface_limits)
  target_link_libraries(test_c_api_abi PRIVATE autonomymind_decision_c_api)
  target_include_directories(test_c_api_abi PRIVATE core/include "${CMAKE_CURRENT_BINARY_DIR}/generated")
  target_compile_options(test_c_api_abi PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_compile_definitions(test_c_api_abi PRIVATE FM_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/profiles/autonomymind_core.profile")
  add_test(NAME test_c_api_abi COMMAND test_c_api_abi)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
