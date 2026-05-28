cmake_minimum_required(VERSION 3.10)

# gd32_bridge is cross-compiled for ARM, typically included
# from the parent KF-GINS CMakeLists.txt or built standalone.
# This file can be used standalone with -DCMAKE_TOOLCHAIN_FILE.

project(gd32_bridge CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(gd32_bridge
    main.cpp
    usb_cdc_reader.cpp
    gps_receiver.cpp
    tcp_sender.cpp
)

add_executable(tcp_image_packet_test
    tcp_image_packet_test.cpp
    tcp_image_test_packet.cpp
)

add_executable(tcp_image_test_sender
    tcp_image_test_sender.cpp
    tcp_image_test_packet.cpp
)

target_include_directories(gd32_bridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(tcp_image_packet_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(tcp_image_test_sender PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(gd32_bridge PRIVATE pthread)
