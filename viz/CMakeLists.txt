pkg_check_modules(ASGUARD-VIZ REQUIRED "asguard-viz")
include_directories(${ASGUARD-VIZ_INCLUDE_DIRS})
link_directories(${ASGUARD-VIZ_LIBRARY_DIRS})

pkg_check_modules(ENVIRE-VIZ REQUIRED "envire-viz")
include_directories(${ENVIRE-VIZ_INCLUDE_DIRS})
link_directories(${ENVIRE-VIZ_LIBRARY_DIRS})

set(TARGET_SRC 
    EslamWidget.cpp
    ParticleVisualization.cpp
    )

add_library(eslam-viz SHARED ${TARGET_SRC})
target_link_libraries(eslam-viz
    ${ASGUARD-VIZ_LIBRARIES}
    ${ENVIRE-VIZ_LIBRARIES}
    ${VIZKIT_LIBRARIES}
    )

install(DIRECTORY . DESTINATION include/vizkit FILES_MATCHING PATTERN "*.hpp")
install(TARGETS eslam-viz
    DESTINATION lib)

configure_file(eslam-viz.pc.in eslam-viz.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/eslam-viz.pc
    DESTINATION lib/pkgconfig)
