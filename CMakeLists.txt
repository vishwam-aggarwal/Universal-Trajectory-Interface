cmake_minimum_required(VERSION 3.14)
project(UniversalTrajectoryInterface CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(trajectory
    src/TrapezoidalProfile.cpp
    src/LinePath.cpp
    src/ArcPath.cpp
    src/CartesianMove.cpp
    src/TrajectoryGroup.cpp
)
target_include_directories(trajectory PUBLIC src)

add_executable(test_trapezoidal      tests/test_trapezoidal.cpp)
add_executable(test_trajectory_group tests/test_trajectory_group.cpp)
add_executable(test_cartesian        tests/test_cartesian.cpp)

target_link_libraries(test_trapezoidal      PRIVATE trajectory)
target_link_libraries(test_trajectory_group PRIVATE trajectory)
target_link_libraries(test_cartesian        PRIVATE trajectory)

enable_testing()
add_test(NAME trapezoidal      COMMAND test_trapezoidal)
add_test(NAME trajectory_group COMMAND test_trajectory_group)
add_test(NAME cartesian        COMMAND test_cartesian)
