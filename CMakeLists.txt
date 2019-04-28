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

add_executable(mip
    src/cart_pole_util.cpp
    src/mip_util.cpp
    src/mip_main.cpp)
target_link_libraries(mip
  drake::drake drake::drake-common-text-logging-gflags gflags_shared
)

add_executable(wheel src/wheel_main.cpp)
target_link_libraries(wheel
  drake::drake drake::drake-common-text-logging-gflags gflags_shared
)

add_executable(cart_pole src/cart_pole_main.cpp src/cart_pole_util.cpp)
target_link_libraries(cart_pole
  drake::drake drake::drake-common-text-logging-gflags gflags_shared
)
