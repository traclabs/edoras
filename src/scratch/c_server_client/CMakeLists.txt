cmake_minimum_required(VERSION 3.16)
project(c_server_client)

set(CMAKE_C_STANDARD 99)

add_executable(server_1_udp server_1_udp.c)
add_executable(server_2_udp server_2_udp.c)

