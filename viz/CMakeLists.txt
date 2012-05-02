rock_export_includedir(${PROJECT_SOURCE_DIR}/viz vizkit)

rock_vizkit_plugin(eslam-viz
    MOC BodyContactStateVisualization.cpp ParticleVisualization.cpp PluginLoader.cpp EslamWidget.cpp
    DEPS eslam
    DEPS_PKGCONFIG asguard-viz envire-viz vizkit-base-viz
    HEADERS ParticleVisualization.hpp BodyContactStateVisualization.hpp MapVizEventFilter.hpp EslamWidget.hpp
    )
