cmake_minimum_required(VERSION 2.6)
include($ENV{ROCK_CMAKE_MACROS}/Rock.cmake)
rock_init(eslam 1.0)

rock_export_includedir(${PROJECT_SOURCE_DIR}/src asguard)
rock_standard_layout()
