set(FILTER_HDRS
    ParticleFilter.hpp
    PoseEstimator.hpp
    PoseParticle.hpp
    EmbodiedSlamFilter.hpp
    GaussianMixture.hpp
    ExpectationMaximization.hpp
    Configuration.hpp
    )

set(FILTER_SRCS
    PoseEstimator.cpp
    EmbodiedSlamFilter.cpp
    )

option( USE_OPENMP "Switch on OpenMP for this library." )
if( USE_OPENMP )
    find_package( OpenMP )
    add_definitions( ${OpenMP_CXX_FLAGS} -DUSE_OPENMP )
    set(OpenMP_LIBRARIES gomp)
endif( USE_OPENMP )

add_library(eslam SHARED ${FILTER_SRCS})
target_link_libraries(eslam 
    ${OpenMP_LIBRARIES}
    ${ASGUARD_LIBRARIES}
    ${ENVIRE_LIBRARIES} )
install(TARGETS eslam
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)

install(FILES ${FILTER_HDRS} DESTINATION include/eslam)
