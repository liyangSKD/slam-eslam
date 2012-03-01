rock_export_includedir(${PROJECT_SOURCE_DIR}/viz vizkit)

rock_vizkit_plugin(eslam-viz
    EslamWidget.cpp
    MOC BodyContactStateVisualization.cpp ParticleVisualization.cpp
    DEPS eslam
    DEPS_PKGCONFIG asguard-viz envire-viz vizkit-base-viz
    HEADERS EslamWidget.hpp ParticleVisualization.hpp BodyContactStateVisualization.hpp
    )
