cmake_minimum_required(VERSION 3.14)
project(unitree_test)

set(ROBOT_TYPE Go1)         # The type of robot, support Go1 and A1 currently
set(PLATFORM amd64)         # The platform to compile, support amd64 and arm64

set(REAL_ROBOT ON)          # Link real robot or not, ON or OFF
set(DEBUG OFF)              # Use debug functions or not, ON or OFF

if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()
if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

set(CMAKE_CXX_STANDARD 11)
find_package(Boost)

if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python2 COMPONENTS Interpreter Development NumPy)
endif()

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")

if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        include_directories(
            library/unitree_legged_sdk_3.2/include
        )
        link_directories(
            library/unitree_legged_sdk_3.2/lib
        )
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
        include_directories(
            library/unitree_legged_sdk-3.8.0/include
        )
        if(${PLATFORM} STREQUAL "amd64")
            link_directories(
                library/unitree_legged_sdk-3.8.0/lib/cpp/amd64
            )
        elseif(${PLATFORM} STREQUAL "arm64")
            link_directories(
                library/unitree_legged_sdk-3.8.0/lib/cpp/arm64
            )
        endif()
    endif()
endif()


# aux_source_directory(src SRC_LIST)
file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

add_executable(junior_ctrl_test src/main.cpp ${SRC_LIST})
if(DEBUG)
    target_include_directories(junior_ctrl_test PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
    target_link_libraries(junior_ctrl_test Python2::Python Python2::NumPy)
endif()
if(REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        if(${PLATFORM} STREQUAL "amd64")
            target_link_libraries(junior_ctrl_test libunitree_legged_sdk_amd64.so)
        elseif(${PLATFORM} STREQUAL "arm64")
            target_link_libraries(junior_ctrl_test libunitree_legged_sdk_arm64.so)
        endif()
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
            target_link_libraries(junior_ctrl_test libunitree_legged_sdk.a)
    endif()
endif()
target_link_libraries(junior_ctrl_test -pthread lcm)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)