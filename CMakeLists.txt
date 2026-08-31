cmake_minimum_required(VERSION 3.14)
project(UniversalTrajectoryInterface CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Universal-Device-Interface (submodule) ---
# TrajectoryGroup and CartesianMove derive from IDevice. IDevice.cpp holds
# the out-of-line definitions of IDevice's static error-sink storage (C++11
# has no inline variables), so it must be compiled exactly once.
# ITrajectoryProfile/TrapezoidalProfile/IPathGeometry deliberately do NOT
# depend on it -- they stay plain math primitives.
add_library(device extern/Universal-Device-Interface/src/IDevice.cpp)
target_include_directories(device PUBLIC extern/Universal-Device-Interface/src)

add_library(trajectory
    src/TrapezoidalProfile.cpp
    src/SCurveProfile.cpp
    src/JerkPercentProfile.cpp
    src/LinePath.cpp
    src/ArcPath.cpp
    src/CartesianMove.cpp
    src/TrajectoryGroup.cpp
)
target_include_directories(trajectory PUBLIC src)
target_link_libraries(trajectory PUBLIC device)

add_executable(test_trapezoidal      tests/test_trapezoidal.cpp)
add_executable(test_trajectory_group tests/test_trajectory_group.cpp)
add_executable(test_cartesian        tests/test_cartesian.cpp)
add_executable(test_planner_device   tests/test_planner_device.cpp)
add_executable(test_scurve           tests/test_scurve.cpp)
add_executable(test_jerk_percent     tests/test_jerk_percent.cpp)

# The WASM shim compiles with a host compiler too (its EMSCRIPTEN_KEEPALIVE
# macro stubs itself out when __EMSCRIPTEN__ is undefined), so the
# marshalling layer the web app calls into is covered by the same desktop CI
# as everything else. An emsdk toolchain is then only needed to package
# already-tested code -- see wasm/README.md.
add_executable(test_wasm_shim        tests/test_wasm_shim.cpp wasm/uti_wasm.cpp)

target_link_libraries(test_trapezoidal      PRIVATE trajectory)
target_link_libraries(test_trajectory_group PRIVATE trajectory)
target_link_libraries(test_cartesian        PRIVATE trajectory)
target_link_libraries(test_planner_device   PRIVATE trajectory)
target_link_libraries(test_scurve           PRIVATE trajectory)
target_link_libraries(test_jerk_percent     PRIVATE trajectory)
target_link_libraries(test_wasm_shim        PRIVATE trajectory)

enable_testing()
add_test(NAME trapezoidal      COMMAND test_trapezoidal)
add_test(NAME trajectory_group COMMAND test_trajectory_group)
add_test(NAME cartesian        COMMAND test_cartesian)
add_test(NAME planner_device   COMMAND test_planner_device)
add_test(NAME scurve          COMMAND test_scurve)
add_test(NAME jerk_percent    COMMAND test_jerk_percent)
add_test(NAME wasm_shim       COMMAND test_wasm_shim)
