rock_export_includedir(${PROJECT_SOURCE_DIR}/viz vizkit)

rock_vizkit_plugin(eslam-viz
    EslamWidget.cpp
    ParticleVisualization.cpp
    DEPS eslam
    DEPS_PKGCONFIG asguard-viz envire-viz
    )
