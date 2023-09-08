cmake_minimum_required(VERSION 3.8)
project(tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tutorial_interface REQUIRED)

add_executable(
        publisher
        src/publisher.cpp
)
ament_target_dependencies(publisher rclcpp tutorial_interface)

add_executable(
        subscriber
        src/subscriber.cpp
)
ament_target_dependencies(subscriber rclcpp tutorial_interface)

add_executable(
        service_server
        src/service_server.cpp
)
ament_target_dependencies(service_server rclcpp tutorial_interface)

install (TARGETS
        publisher
        service_server
        subscriber
        DESTINATION lib/${PROJECT_NAME}
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
