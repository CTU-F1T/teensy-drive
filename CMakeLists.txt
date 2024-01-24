set(ROS_VERSION $ENV{ROS_VERSION})

if (NOT DEFINED ROS_VERSION OR '${ROS_VERSION} STREQUAL '1)
    set (ROS_VERSION 1)
    cmake_minimum_required(VERSION 2.8.3)
else()
    cmake_minimum_required(VERSION 3.3)
endif()

project(teensy_drive)

# TODO: conditional setting of CMAKE_C_STANDARD does not work on macOS 10.14.6, cmake 3.21.4
# message(ERROR "CMAKE_C_STANDARD = '${CMAKE_C_STANDARD}'")
# Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()
# message(ERROR "CMAKE_C_STANDARD = '${CMAKE_C_STANDARD}'")
set(CMAKE_C_STANDARD 99)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dep_pkgs
    std_msgs
    geometry_msgs
    nav_msgs
    tf2
)

set(dep_ros1_pkgs
    f1tenth_race
)

set(dep_ros2_pkgs
    teensy_drive_msgs
)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
if (ROS_VERSION EQUAL 1)
add_definitions(-DROS1_BUILD)
find_package(catkin REQUIRED
    roscpp
    ${dep_pkgs}
    ${dep_ros1_pkgs}
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    ${dep_pkgs}
    ${dep_ros1_pkgs}
)

add_executable(${PROJECT_NAME}_node
  "src/crc.c"
  "src/protocol.c"
  "src/serial_port.cpp"
  "src/main.cpp"
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

elseif (ROS_VERSION EQUAL 2)
add_definitions(-DROS2_BUILD)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

foreach (pkg ${dep_pkgs})
    find_package(${pkg} REQUIRED)
endforeach()

foreach (pkg ${dep_ros2_pkgs})
    find_package(${pkg} REQUIRED)
endforeach()

add_executable(${PROJECT_NAME}_node
  "src/crc.c"
  "src/protocol.c"
  "src/serial_port.cpp"
  "src/main.cpp"
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  ${dep_pkgs}
  ${dep_ros2_pkgs}
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

else()
    message(FATAL_ERROR "ROS_VERSION is not set.")
endif()
