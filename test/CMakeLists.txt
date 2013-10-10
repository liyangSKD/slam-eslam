rock_find_pkgconfig(asguard asguard)
if (asguard_FOUND)
    rock_testsuite(unit_test UnitTest.cpp 
        DEPS eslam)

    rock_testsuite(test_contact_model testContactModel.cpp
        DEPS eslam)

    if( vizkit3d_FOUND )
        rock_testsuite(testWidget testWidget.cpp
            DEPS eslam-viz 
            DEPS_PKGCONFIG envire vizkit3d)

        rock_testsuite(testMap testMap.cpp
            DEPS eslam-viz 
            DEPS_PKGCONFIG envire vizkit3d)

    endif( vizkit3d_FOUND )
endif()
