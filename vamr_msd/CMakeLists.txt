cmake_minimum_required(VERSION 3.5)
project(vamr_msd)

# Default C standard
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

# ------------------------------------------------------------------------------
# Install Project Resources
# ------------------------------------------------------------------------------

install(
  DIRECTORY
    config
    description
    launch
    maps
    worlds
  DESTINATION
    share/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Install Python Executable Nodes
# ------------------------------------------------------------------------------

install(
  PROGRAMS
    scripts/goal_queue_node.py
    scripts/vision_assist_node.py
  DESTINATION
    lib/${PROJECT_NAME}
)

# ------------------------------------------------------------------------------
# Testing
# ------------------------------------------------------------------------------

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  # Uncomment if copyright headers are not added yet
  # set(ament_cmake_copyright_FOUND TRUE)

  # Uncomment if package is not inside a git repository
  # set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()

ament_package()