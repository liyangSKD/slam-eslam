project(particle_filter)
set(PROJECT_VERSION 1.0)

cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(src)
add_executable(unit_test ${TEST_SRC} test/UnitTest.cpp)

set(FILTER_HDRS
    src/ParticleFilter.hpp
    )

install(FILES ${FILTER_HDRS} DESTINATION include/particle_filter)

configure_file(Doxyfile.in Doxyfile @ONLY)
add_custom_target(doc doxygen Doxyfile)

configure_file(particle_filter.pc.in particle_filter.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/particle_filter.pc DESTINATION lib/pkgconfig)

