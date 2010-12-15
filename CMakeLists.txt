project(eslam)
set(PROJECT_VERSION 1.0)

cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)
add_definitions (-Wall)

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

pkg_check_modules(VIZKIT "vizkit")
include_directories(${VIZKIT_INCLUDE_DIRS})
link_directories(${VIZKIT_LIBRARY_DIRS})

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(BEFORE .)
add_subdirectory(eslam)
if( VIZKIT_FOUND )
    add_subdirectory(viz)
    add_subdirectory(test)
endif( VIZKIT_FOUND )

configure_file(Doxyfile.in Doxyfile @ONLY)
add_custom_target(doc doxygen Doxyfile)

configure_file(eslam.pc.in eslam.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/eslam.pc DESTINATION lib/pkgconfig)

