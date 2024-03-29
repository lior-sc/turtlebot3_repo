cmake_minimum_required(VERSION 3.5)
project(turtlebot3_joy)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/turtlebot3_joy/turtlebot3_joy_node.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# joy_teleop executable
set(executable_name ${PROJECT_NAME}_node)

add_executable(${executable_name} src/turtlebot3_joy_main.cpp)
ament_target_dependencies(${executable_name} ${dependencies})
target_link_libraries(${executable_name} ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directory in share/${project_name} so that we can use the files when we source the project
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()