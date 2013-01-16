rock_find_pkgconfig(asguard asguard)
if (asguard_FOUND)
    rock_testsuite(unit_test UnitTest.cpp 
        DEPS eslam)

    rock_testsuite(test_contact_model testContactModel.cpp
        DEPS eslam)

    if( vizkit_FOUND )
        rock_testsuite(testWidget testWidget.cpp
            DEPS eslam-viz 
            DEPS_PKGCONFIG envire vizkit)

        rock_testsuite(testMap testMap.cpp
            DEPS eslam-viz 
            DEPS_PKGCONFIG envire vizkit)

    endif( vizkit_FOUND )
endif()
