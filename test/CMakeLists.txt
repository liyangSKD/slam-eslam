rock_testsuite(unit_test UnitTest.cpp 
    DEPS eslam)

rock_testsuite(test_contact_model testContactModel.cpp
    DEPS eslam)

if( vizkit_FOUND )
    rock_testsuite(testWidget testWidget.cpp
	DEPS eslam-viz 
	DEPS_PKGCONFIG asguard envire vizkit)

endif( vizkit_FOUND )
