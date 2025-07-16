cmake_minimum_required(VERSION 3.8)
project(quadruped)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)        # 升级到C++17
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 基础配置
add_compile_options(-Wno-register)

# ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial_driver REQUIRED CONFIG)  # 添加CONFIG模式
find_package(asio_cmake_module REQUIRED)     # 添加asio模块
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 添加消息生成
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCmdMsg.msg"
  "msg/MotorStateMsg.msg"
  DEPENDENCIES std_msgs
)

find_package(serial_driver REQUIRED CONFIG QUIET)
if(NOT serial_driver_FOUND)
    message(FATAL_ERROR "serial_driver package not found!")
else()
    message(STATUS "Found serial_driver package at: ${serial_driver_DIR}")
endif()

include_directories(
    /opt/ros/jazzy/include
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c  
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
    include
    include/FSM
    include/control
    include/Gait
    include/common
    include/interface
    include/serial_xm
)

# 源文件配置
file(GLOB_RECURSE COMM_SRC
    "src/serial_xm/*.cpp"
)

file(GLOB_RECURSE CTRL_SRC
    "src/main.cpp"
    "src/control/*.cpp"
    "src/FSM/*.cpp"
    "src/common/*.cpp"
    "src/interface/*.cpp"
    "src/Gait/*.cpp"
    "src/*/*.cc"  # 确保包含主函数文件
    "src/serial_xm/*.cpp"
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")  # 强制添加调试符号
endif()

# 可执行文件配置
# add_executable(robot_comm ${COMM_SRC})
add_executable(main_controller 
                ${CTRL_SRC}
                ${COMM_SRC}
    )


# 链接配置
foreach(TARGET main_controller)
    target_link_libraries(${TARGET}
        rclcpp::rclcpp
        ${serial_driver_LIBRARIES}
        ${PROJECT_NAME}__rosidl_typesupport_c 
        ${PROJECT_NAME}__rosidl_typesupport_cpp
        pthread
        rt
    )
    target_compile_options(${TARGET} PRIVATE -g -O0)
    ament_target_dependencies(${TARGET}
        rclcpp
        sensor_msgs
        serial_driver
        asio_cmake_module
        std_srvs
    )
endforeach()

# 安装配置
install(TARGETS 
    # robot_comm
    main_controller
    DESTINATION lib/${PROJECT_NAME}  # 确保安装到正确目录
)

install(DIRECTORY 
    include/
    launch/
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()