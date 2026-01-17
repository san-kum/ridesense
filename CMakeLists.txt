cmake_minimum_required(VERSION 3.20)
project(ridersense VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)
find_package(Threads REQUIRED)

add_library(ridersense_common
    src/common/config.cpp
    src/common/logger.cpp
)

add_library(ridersense_sync
    src/sync/timestamp_sync.cpp
    src/sync/resampler.cpp
    src/sync/time_aligner.cpp
)

target_include_directories(ridersense_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(ridersense_sync PUBLIC
  ridersense_common
  ridersense_capture
)

target_link_libraries(ridersense_common PUBLIC
    spdlog::spdlog
    yaml-cpp
)

add_library(ridersense_capture
    src/capture/camera_stream.cpp
    src/capture/imu_stream.cpp
    src/capture/gps_stream.cpp
)

target_link_libraries(ridersense_capture PUBLIC
    ridersense_common
    ${OpenCV_LIBS}
    Threads::Threads
)

add_library(ridersense_storage
    src/storage/frame_logger.cpp
)

target_link_libraries(ridersense_storage PUBLIC
    ridersense_common
    ridersense_capture
)

add_executable(ridersense
    src/main.cpp
)

target_link_libraries(ridersense PRIVATE
    ridersense_common
    ridersense_capture
    ridersense_storage
    ridersense_sync
)

