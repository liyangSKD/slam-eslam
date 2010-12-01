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

find_package(Eigen2 REQUIRED)
include_directories(${EIGEN2_INCLUDE_DIR})

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(src)
add_executable(unit_test ${TEST_SRC} test/UnitTest.cpp)

set(FILTER_HDRS
    src/ParticleFilter.hpp
    src/PoseEstimator.hpp
    src/PoseParticle.hpp
    src/EmbodiedSlamFilter.hpp
    )

set(FILTER_SRCS
    src/PoseEstimator.cpp
    src/EmbodiedSlamFilter.cpp
    )

if( USE_OPENMP )
    set(CCMAKE_CXX_FLAGS -fopenmp)
    set(CCMAKE_C_FLAGS -fopenmp)
    set(CMAKE_EXE_LINKER_FLAGS -lgomp)
endif( USE_OPENMP )

add_library(eslam SHARED ${FILTER_SRCS})
target_link_libraries(eslam ${ASGUARD_LIBRARIES})
install(TARGETS eslam
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)

install(FILES ${FILTER_HDRS} DESTINATION include/eslam)

configure_file(Doxyfile.in Doxyfile @ONLY)
add_custom_target(doc doxygen Doxyfile)

configure_file(eslam.pc.in eslam.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/eslam.pc DESTINATION lib/pkgconfig)

