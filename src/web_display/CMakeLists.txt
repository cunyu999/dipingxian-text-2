cmake_minimum_required(VERSION 3.5)
project(web_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_camera_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include)

add_executable(web_display src/web_display.cpp)
ament_target_dependencies(web_display rclcpp my_camera_msgs)

find_package(JPEG REQUIRED)
target_link_libraries(web_display JPEG::JPEG)

install(TARGETS
  web_display
  DESTINATION lib/${PROJECT_NAME})

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
