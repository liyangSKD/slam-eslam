add_executable(unit_test UnitTest.cpp)


if( VIZKIT_FOUND )
    add_executable(testWidget testWidget.cpp)
    target_link_libraries(testWidget eslam-viz eslam
	${ASGUARD_LIBRARIES}
	${ENVIRE_LIBRARIES}
	${VIZKIT_LIBRARIES}
	)
    
    add_executable(test_gmm GMMEM.cpp)
    target_link_libraries(test_gmm 
	${VIZKIT_LIBRARIES}
	)
endif( VIZKIT_FOUND )
