add_executable(unit_test UnitTest.cpp)

if( ENVIEW_FOUND )
    find_package(OpenSceneGraph REQUIRED osgManipulator osgViewer osgFX)
    include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
    link_directories(${OPENSCENEGRAPH_LIBRARY_DIRS})

    find_package(Qt4 REQUIRED)
    set(QT_USE_OPENGL TRUE)
    include(${QT_USE_FILE})

    add_executable(testWidget testWidget.cpp)
    target_link_libraries(testWidget eslam-viz eslam
	${ASGUARD_LIBRARIES}
	${ENVIRE_LIBRARIES}
	${ENVIEW_LIBRARIES}
	${QT_LIBRARIES} 
	${QT_QTOPENGL_LIBRARIES} 
	${OPENSCENEGRAPH_LIBRARIES}
	)
endif( ENVIEW_FOUND )
