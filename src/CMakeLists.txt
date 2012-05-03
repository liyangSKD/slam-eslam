set(FILTER_HDRS
    ParticleFilter.hpp
    PoseEstimator.hpp
    PoseParticle.hpp
    ContactModel.hpp
    ContactState.hpp
    ContactOdometry.hpp
    EmbodiedSlamFilter.hpp
    Configuration.hpp
    SurfaceHash.hpp
    )

set(FILTER_SRCS
    PoseEstimator.cpp
    ContactModel.cpp
    EmbodiedSlamFilter.cpp
    ContactOdometry.cpp
    )

rock_library(eslam
    SOURCES ${FILTER_SRCS}
    HEADERS ${FILTER_HDRS}
    DEPS_PKGCONFIG base-types envire asguard terrain_estimator)
    
option( USE_OPENMP "Switch on OpenMP for this library." )
if( USE_OPENMP )
    find_package( OpenMP )
    add_definitions( ${OpenMP_CXX_FLAGS} -DUSE_OPENMP )
    set(OpenMP_LIBRARIES gomp)
endif( USE_OPENMP )

target_link_libraries(eslam ${OpenMP_LIBRARIES})
