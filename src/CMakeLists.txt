#之后此处需要根据用途分类进行链接，整理
add_library(front_end 
    wifi.cpp
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    visual_odometry.cpp
    converter.cpp
    data.cpp
    openni.cpp
)

target_link_libraries( front_end
    ${THIRD_PARTY_LIBS}
)
