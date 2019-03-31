cmake_minimum_required(VERSION 3.0.0)
#set(CMAKE_BUILD_TYPE Debug)
project(tribot)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

find_package(drake CONFIG REQUIRED)
find_package(GFlags MODULE REQUIRED COMPONENTS shared)

include_directories(include)

#find_package (Threads REQUIRED)

add_executable(tribot src/main.cpp)
target_link_libraries(tribot
  drake::drake drake::drake-common-text-logging-gflags gflags_shared
)

add_executable(segway src/segway_main.cpp)
target_link_libraries(segway
  drake::drake drake::drake-common-text-logging-gflags gflags_shared
)

