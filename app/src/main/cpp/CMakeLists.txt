cmake_minimum_required(VERSION 3.22.1)
project("assistivenav" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(
    -O3
    -ffast-math
    -march=armv8-a
    -fvisibility=hidden
)

# calib3d is now required for solvePnPRansac and triangulatePoints.
find_package(OpenCV REQUIRED core imgproc video calib3d)

if(NOT DEFINED OPENAL_DIR)
    message(FATAL_ERROR
        "OPENAL_DIR is not set.\n"
        "Add 'openal.dir=/path/to/openal-soft' to local.properties.")
endif()

add_library(openal SHARED IMPORTED)
set_target_properties(openal PROPERTIES
    IMPORTED_LOCATION
        "${OPENAL_DIR}/build-android/${ANDROID_ABI}/libopenal.so")

add_library(assistivenav SHARED
    FlowEngine.cpp
    GridAnalyzer.cpp
    FlowClassifier.cpp
    ImuFusion.cpp
    ObstacleTracker.cpp
    AudioEngine.cpp
    PTAMSystem.cpp      # tracking + background mapping
    VoxelMap.cpp        # 3-D occupancy grid
    SoundMapper.cpp     # voxel clusters → audio descriptors
    jni_bridge.cpp
)

target_include_directories(assistivenav PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${OPENAL_DIR}/include
)

target_link_libraries(assistivenav
    android
    log
    ${OpenCV_LIBS}
    openal
)