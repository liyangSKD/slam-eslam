project(eslam)
set(PROJECT_VERSION 1.0)

cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

pkg_check_modules(BASE REQUIRED "base-types")
include_directories(${BASE_INCLUDE_DIRS})
link_directories(${BASE_LIBRARY_DIRS})

pkg_check_modules(ASGUARD REQUIRED "asguard")
include_directories(${ASGUARD_INCLUDE_DIRS})
link_directories(${ASGUARD_LIBRARY_DIRS})

pkg_check_modules(ENVIRE "envire")
include_directories(${ENVIRE_INCLUDE_DIRS})
link_directories(${ENVIRE_LIBRARY_DIRS})

pkg_check_modules(ENVIEW "enview")
include_directories(${ENVIEW_INCLUDE_DIRS})
link_directories(${ENVIEW_LIBRARY_DIRS})

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(BEFORE .)
add_subdirectory(eslam)
add_subdirectory(viz)
add_subdirectory(test)

configure_file(Doxyfile.in Doxyfile @ONLY)
add_custom_target(doc doxygen Doxyfile)

configure_file(eslam.pc.in eslam.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/eslam.pc DESTINATION lib/pkgconfig)

