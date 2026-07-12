cmake_minimum_required(VERSION 3.14)
project(UniversalTrajectoryInterface CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(trajectory
    src/TrapezoidalProfile.cpp
)
target_include_directories(trajectory PUBLIC src)

add_executable(test_trapezoidal
    tests/test_trapezoidal.cpp
)
target_link_libraries(test_trapezoidal PRIVATE trajectory)

enable_testing()
add_test(NAME trapezoidal COMMAND test_trapezoidal)
