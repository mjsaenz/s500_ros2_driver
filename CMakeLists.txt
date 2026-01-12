cmake_minimum_required(VERSION 3.8)
project(s500_ros2_driver)

# === START DEBUG BLOCK ===
# This block will print information during the build process to help us debug.
message(STATUS "--- DEBUGGING CMAKE ---")
message(STATUS "Project Name variable is: [${PROJECT_NAME}]")
message(STATUS "Package source directory is: [${CMAKE_CURRENT_SOURCE_DIR}]")
message(STATUS "The include path being added is: [${CMAKE_CURRENT_SOURCE_DIR}/include]")

# Let's have CMake itself check if it can find the file at the expected location.
set(EXPECTED_HEADER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/device/ping-device-s500.hpp")
message(STATUS "CMake is checking for the existence of this exact file: [${EXPECTED_HEADER_PATH}]")
if(EXISTS ${EXPECTED_HEADER_PATH})
    message(STATUS "SUCCESS: CMake reports that the header file EXISTS at the expected path.")
else()
    message(FATAL_ERROR "FAILURE: CMake reports the header file DOES NOT EXIST at the path. Please verify the path, including spelling and capitalization.")
endif()
message(STATUS "--- END DEBUGGING CMAKE ---")
# === END DEBUG BLOCK ===


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/S500Profile6T.msg"
  "msg/S500Distance2.msg"
  DEPENDENCIES std_msgs
)

add_executable(s500_publisher_node src/s500_node.cpp)

target_include_directories(s500_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SORUCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(s500_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(s500_publisher_node
  rclcpp
  std_msgs
  rcl_interfaces
)

target_link_libraries(s500_publisher_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY rosidl_generator_cpp/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS s500_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)  

install(
  DIRECTORY launch
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

ament_package()
