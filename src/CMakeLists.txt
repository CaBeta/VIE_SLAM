add_library( vie_slam SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    visual_odometry.cpp
)

target_link_libraries( vie_slam
    ${THIRD_PARTY_LIBS}
)
