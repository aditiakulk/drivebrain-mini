# file tells cmake how to build program
cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)
project(mini-drivebrain)

find_package(Protobuf REQUIRED CONFIG)
find_package(foxglove-websocket REQUIRED)
find_package(mcap REQUIRED)
find_package(spdlog REQUIRED)
find_package(Threads REQUIRED)

# Add your source files here
add_executable(mini-drivebrain
    drivebrain_app/main.cpp
    drivebrain_comms/src/FlowmeterComms.cpp
    drivebrain_core/src/FoxgloveServer.cpp
    drivebrain_core/src/MCAPLogger.cpp
    proto/drivebrain_msgs.proto
)

# proto/drivebrain_msgs.proto -> #include <drivebrain_msgs.pb.h>
protobuf_generate(TARGET mini-drivebrain IMPORT_DIRS proto)

target_include_directories(mini-drivebrain PRIVATE drivebrain_core/include drivebrain_comms/include ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(mini-drivebrain PRIVATE
    foxglove-websocket::foxglove-websocket
    mcap::mcap
    spdlog::spdlog
    protobuf::libprotobuf
    Threads::Threads
)

# static so the binary runs on the pi without installing anything
target_link_options(mini-drivebrain PRIVATE -static)