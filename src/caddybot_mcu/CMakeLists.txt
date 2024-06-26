cmake_minimum_required(VERSION 3.8)
project(caddybot_mcu LANGUAGES C)

project(caddybot_mcu LANGUAGES C)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(caddybot_msgs REQUIRED)

add_executable(mcu src/main.c)

ament_target_dependencies(mcu
  rcl
  rclc
  std_msgs
  sensor_msgs
  nav_msgs
  caddybot_msgs
  )

install(TARGETS mcu
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch
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
