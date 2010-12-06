set(FILTER_HDRS
    ParticleFilter.hpp
    PoseEstimator.hpp
    PoseParticle.hpp
    EmbodiedSlamFilter.hpp
    )

set(FILTER_SRCS
    PoseEstimator.cpp
    EmbodiedSlamFilter.cpp
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
