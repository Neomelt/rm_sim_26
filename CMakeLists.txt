cmake_minimum_required(VERSION 3.24)
project(rm_sim_26)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install directories (skip missing ones)
set(_install_dirs launch worlds moudles rviz config)
foreach(_dir IN LISTS _install_dirs)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}")
    install(
      DIRECTORY ${_dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

install(
  PROGRAMS scripts/laserscan_to_pointcloud.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()