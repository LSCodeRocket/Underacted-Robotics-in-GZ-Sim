cmake_minimum_required(VERSION 3.10)
project(simple_pendulum_controller)

find_package(gz-sim8 REQUIRED)

add_library(SimplePendulumController SHARED PID_Pendulum.cc)
target_link_libraries(SimplePendulumController gz-sim8::gz-sim8)
target_include_directories(SimplePendulumController PUBLIC ${GZ_SIM8_INCLUDE_DIRS})