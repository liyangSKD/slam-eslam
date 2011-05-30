rock_testsuite(unit_test UnitTest.cpp 
    DEPS eslam)

if( vizkit_FOUND )
    rock_testsuite(testWidget testWidget.cpp
	DEPS eslam-viz 
	DEPS_PKGCONFIG asguard envire vizkit)

    rock_testsuite(test_gmm GMMEM.cpp
	DEPS_PKGCONFIG vizkit)
endif( vizkit_FOUND )
