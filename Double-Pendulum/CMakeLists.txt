cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(gz-cmake3 REQUIRED)

project(simple_pendulum_controller)

gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})


set(LIBRARIES FeedbackCancellation PIDController DataLogger)


foreach(LIB IN LISTS LIBRARIES)

  add_library(${LIB} SHARED src/${LIB}.cc)
  set_property(TARGET ${LIB} PROPERTY CXX_STANDARD 17)
  target_link_libraries(${LIB}
    PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

endforeach()
