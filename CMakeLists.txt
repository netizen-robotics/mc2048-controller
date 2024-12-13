cmake_minimum_required(VERSION 3.16)
project(mc2408_controller LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  tf2
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# # COMPILE
add_library(
  mc2408_controller
  SHARED
  src/controller.cpp
  src/mc2408_handler.cpp
  src/types.cpp
  src/utils.cpp
)
target_compile_features(mc2408_controller PUBLIC cxx_std_17)
target_include_directories(mc2408_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/include>
  $<INSTALL_INTERFACE:include/mc2408_controller>
)

ament_target_dependencies(
  mc2408_controller PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(mc2408_controller PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "NETIZEN_ROBOTICS_MC2048_CONTROLLER_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface interface.xml)

install(
  DIRECTORY src/include/
  DESTINATION include/
)
install(TARGETS mc2408_controller
  EXPORT export_mc2408_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_targets(export_mc2408_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
