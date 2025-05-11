cmake_minimum_required(VERSION 3.10.0)
project(manipulator_control VERSION 0.1.0 LANGUAGES C CXX)

find_package(pinocchio REQUIRED)

set(WEBOTS_HOME "/usr/local/webots")
set(WEBOTSLIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})

add_executable(manipulator_control
    main.cpp
    inverse_kinematics.cpp
    webots_interface.cpp)

target_link_directories(manipulator_control PRIVATE ${WEBOTS_HOME}/lib/controller)
target_include_directories(manipulator_control PRIVATE ${WEBOTS_HOME}/include/controller/cpp)

target_link_libraries(manipulator_control PRIVATE
    pinocchio::pinocchio
    ${WEBOTSLIBRARIES})

