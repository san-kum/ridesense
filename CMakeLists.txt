cmake_minimum_required(VERSION 3.20)
project(ridesense)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Dependencies
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

# Common library
add_library(ridersense_common
    src/common/config.cpp
    src/common/logger.cpp
)

target_include_directories(ridersense_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(ridersense_common PUBLIC
    spdlog::spdlog
    yaml-cpp
)

# Main executable
add_executable(ridersense
    src/main.cpp
)

target_link_libraries(ridersense PRIVATE
    ridersense_common
)
