cmake_minimum_required(VERSION 3.8)
project(platoon_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install only the directories that actually exist in this package.
# 'platoon_description' only contains launch files — urdf and models
# belong to the 'br2_robodog' package, not here.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()