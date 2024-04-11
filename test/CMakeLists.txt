SET(TEST_SOURCES test_triangulation)

FOREACH (test_src ${TEST_SOURCES})
    ADD_EXECUTABLE(${test_src} ${test_src}.cpp
            ../include/myslam/ORBextractor.h
            ../include/myslam/deeplcd.h
    )
    TARGET_LINK_LIBRARIES(${test_src} ${THIRD_PARTY_LIBS} myslam)
    ADD_TEST(${test_src} ${test_src})
ENDFOREACH (test_src)
