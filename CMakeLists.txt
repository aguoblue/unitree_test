cmake_minimum_required(VERSION 3.14)
project(unitree_test)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置机器人类型和平台
set(ROBOT_TYPE Go1)         # 支持 Go1 和 A1
set(PLATFORM amd64)         # 支持 amd64 和 arm64
set(REAL_ROBOT ON)          # 链接真实机器人
set(DEBUG OFF)              # 调试模式

# 检查必需变量
if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()
if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

# 设置机器人类型宏
if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()


# 查找依赖
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake") # 假设 SDK 安装路径
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED)

# 调试目标属性
function(print_target_properties target)
    if(TARGET ${target})
        get_target_property(INCLUDE_DIRS ${target} INTERFACE_INCLUDE_DIRECTORIES)
        get_target_property(LIBRARIES ${target} IMPORTED_LOCATION)
        message(STATUS "${target} INTERFACE_INCLUDE_DIRECTORIES: ${INCLUDE_DIRS}")
        message(STATUS "${target} IMPORTED_LOCATION: ${LIBRARIES}")
    else()
        message(STATUS "${target} is not a target")
    endif()
endfunction()

print_target_properties(unitree_sdk2)

# 调试模式
if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python2 COMPONENTS Interpreter Development NumPy)
endif()

# 真实机器人支持
if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
endif()

# 包含头文件目录
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
)

# 设置优化标志
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# aux_source_directory(src SRC_LIST)
file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

add_executable(junior_ctrl_test src/main.cpp ${SRC_LIST})

# 链接库
target_link_libraries(junior_ctrl_test
    ${Boost_LIBRARIES}
    -pthread
    lcm
    unitree_sdk2
)


if(DEBUG)
    target_include_directories(junior_ctrl_test PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
    target_link_libraries(junior_ctrl_test Python2::Python Python2::NumPy)
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)