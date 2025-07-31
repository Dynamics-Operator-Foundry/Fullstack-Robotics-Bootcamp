cmake_minimum_required(VERSION 3.10)
project(kinematics)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED) 

add_executable(ctrl_test 
    src/ctrl_test.cpp
    src/com_util.cpp
)

target_link_libraries(ctrl_test
    ${CMAKE_THREAD_LIBS_INIT}
)

add_executable(forward_kinematics 
    src/forward_kinematics.cpp
    src/com_util.cpp
)

target_link_libraries(forward_kinematics
    ${CMAKE_THREAD_LIBS_INIT}
)
