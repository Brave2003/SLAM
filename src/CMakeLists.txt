add_library(myslam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp
        savefile.cpp
        loopclosure.cpp
        ORBextractor.cpp
        deeplcd.cpp

)


target_link_libraries(myslam
        ${THIRD_PARTY_LIBS}
        )
