cmake_minimum_required(VERSION 3.16)
project(cpp_server_client)

add_executable(server_udp server_udp.cpp)
#target_include_directories(server_udp )

add_executable(client_udp client_udp.cpp)

add_executable(server_1_udp server_1_udp.cpp)
add_executable(server_2_udp server_2_udp.cpp)

