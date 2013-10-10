rock_export_includedir(${PROJECT_SOURCE_DIR}/viz vizkit3d)

rock_vizkit_plugin(eslam-viz
    MOC ParticleVisualization.cpp PluginLoader.cpp EslamWidget.cpp
    DEPS eslam
    DEPS_PKGCONFIG envire-viz base-viz
    HEADERS ParticleVisualization.hpp MapVizEventFilter.hpp EslamWidget.hpp
    )
