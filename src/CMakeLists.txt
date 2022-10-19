add_library(kinect-dev STATIC ./kinect_log.cpp ./volumetric_video.cpp ./kinect_mkv2_volumetric_video.cpp)
target_link_libraries(kinect-dev k4a k4arecord depthengine_2_0 turbojpeg)
