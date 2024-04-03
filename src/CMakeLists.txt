add_library(slam SHARED
    backend.cpp
    camera.cpp
    config.cpp
    dataset.cpp
    frame.cpp
    frontend.cpp
    map.cpp
    mappoint.cpp
    viewer.cpp
    visual_odometry.cpp
)

target_link_libraries(slam ${THIRD_PARTY_LIBS})