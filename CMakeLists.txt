cmake_minimum_required(VERSION 3.8)
project(remap_manager)

# Global flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "/usr/lib/x86_64-linux-gnu/cmake/OpenVDB")

find_package(ament_cmake_auto REQUIRED)
find_package(OpenVDB MODULE REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/remap_manager/remap_manager.cpp
  src/remap_manager/semantic_remap_manager.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenVDB_LIBRARIES}
)

ament_auto_add_executable(remap_manager_node
  src/remap_manager_node.cpp
)

target_link_libraries(remap_manager_node
  ${PROJECT_NAME}_lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}_lib
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
