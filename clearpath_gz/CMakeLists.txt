cmake_minimum_required(VERSION 3.20)
project(clearpath_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config geotif launch meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(TOP_LEVEL_TEST_FILE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../unit_test/clearpath_gz/test_robot_spawn_launch.py"
  )
  if(EXISTS "${TOP_LEVEL_TEST_FILE}")
    set(ROBOT_SPAWN_TEST_FILE "${TOP_LEVEL_TEST_FILE}")
  else()
    set(ROBOT_SPAWN_TEST_FILE "test/test_robot_spawn_launch.py")
  endif()

  ament_add_pytest_test(
    test_robot_spawn_launch
    "${ROBOT_SPAWN_TEST_FILE}"
  )
endif()

ament_package()
