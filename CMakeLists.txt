cmake_minimum_required(VERSION 3.5)
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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(teensy_drive_msgs REQUIRED)

# add teensy_drive node executable
add_executable(teensy_drive
  "src/crc.c"
  "src/protocol.c"
  "src/serial_port.cpp"
  "src/main.cpp"
)
ament_target_dependencies(teensy_drive
  rclcpp
  std_msgs
  teensy_drive_msgs
)

# install launch files
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# install nodes
install(
  TARGETS teensy_drive
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
